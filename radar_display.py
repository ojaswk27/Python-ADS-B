#!/usr/bin/env python3
"""
ADS-B Radar Display
===================
Live PPI radar receiver — rotating sweep, fading trails, track panel.

Usage
-----
    python radar_display.py
    python radar_display.py --centre 51.5,-0.5 --range 150
"""

import argparse
import math
import socket
import struct
import threading
import time
import tkinter as tk
from collections import deque

import net_config
from adsb_decoder import Aircraft, decode_message


# ── Palette (monochrome) ──────────────────────────────────────────────────────

_BG      = "#000000"
_RADAR   = "#050505"
_RING_D  = "#1c1c1c"
_RING_B  = "#444444"
_SWEEP   = "#ffffff"
_SWEEP_T = "#1a1a1a"
_DIM     = "#444444"
_PANEL   = "#0c0c0c"
_FG      = "#cccccc"
_FG_DIM  = "#555555"
_SEP     = "#1e1e1e"
_ENTRY   = "#111111"
_BTN     = "#1a1a1a"
_BTN_ACT = "#2a2a2a"

# Blip colours by age since last illumination
_B_FRESH = "#ffffff"   # 0 – 3 s
_B_MED   = "#888888"   # 3 – 10 s
_B_OLD   = "#444444"   # 10 – 30 s

# Trail dots (newest → oldest)
_TRAIL = ["#444444", "#3a3a3a", "#303030", "#262626",
          "#1e1e1e", "#181818", "#121212", "#0e0e0e"]

_CANVAS_SZ = 680
_PANEL_W   = 200
_SWEEP_SPD = 36.0


# ── Coordinate helper ─────────────────────────────────────────────────────────

def _ll_to_xy(lat, lon, cx, cy, r, c_lat, c_lon, rng):
    s    = r / rng
    nm_e = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    nm_n = (lat - c_lat) * 60.0
    if math.hypot(nm_e, nm_n) > rng * 1.02:
        return None
    return cx + nm_e * s, cy - nm_n * s


# ── App ───────────────────────────────────────────────────────────────────────

def _sep(parent):
    tk.Frame(parent, bg=_SEP, height=1).pack(fill=tk.X, padx=0, pady=4)


class App(tk.Tk):
    """
    Radar display. RX thread decodes into self._fleet; main loop draws at 20 fps.
    All tkinter calls are main-thread only — thread writes only to self._fleet
    (under lock) and self._rx_status (plain string).
    """

    def __init__(self, group, port, iface, c_lat, c_lon, rng):
        super().__init__()
        self.title("Radar Display")
        self.configure(bg=_PANEL)
        self.resizable(False, False)

        self.c_lat, self.c_lon, self.rng = c_lat, c_lon, rng
        self.sweep = 0.0
        self._tick = time.monotonic()

        self._fleet:   dict[str, Aircraft] = {}
        self._history: dict[str, deque]    = {}
        self._illum:   dict[str, float]    = {}
        self._lock     = threading.Lock()
        self._rx_status = "joining…"

        self._build_ui()
        threading.Thread(target=self._rx_loop,
                         args=(group, port, iface), daemon=True).start()
        self._loop()

    # ── geometry ──────────────────────────────────────────────────────────────

    def _geom(self):
        c = _CANVAS_SZ // 2
        return c, c, c - 18

    def _to_xy(self, lat, lon):
        cx, cy, r = self._geom()
        return _ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    # ── UI ────────────────────────────────────────────────────────────────────

    def _entry_row(self, parent, label, var):
        f = tk.Frame(parent, bg=_PANEL)
        f.pack(fill=tk.X, padx=8, pady=1)
        tk.Label(f, text=label, bg=_PANEL, fg=_FG_DIM,
                 font=("Courier", 8), width=9, anchor="w").pack(side=tk.LEFT)
        tk.Entry(f, textvariable=var, width=8,
                 bg=_ENTRY, fg=_FG, insertbackground=_FG,
                 font=("Courier", 9), relief=tk.FLAT, bd=4
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True)

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=_CANVAS_SZ, height=_CANVAS_SZ,
                            bg=_BG, highlightthickness=0, cursor="none")
        self.cv.pack(side=tk.LEFT)

        p = tk.Frame(self, bg=_PANEL, width=_PANEL_W)
        p.pack(side=tk.LEFT, fill=tk.Y)
        p.pack_propagate(False)

        tk.Frame(p, bg=_PANEL, height=10).pack()
        tk.Label(p, text="TRACKS", bg=_PANEL, fg=_FG,
                 font=("Courier", 8), anchor="w").pack(fill=tk.X, padx=8)
        _sep(p)

        self._track_box = tk.Text(
            p, bg=_ENTRY, fg=_FG, font=("Courier", 8),
            relief=tk.FLAT, bd=0, height=14, state=tk.DISABLED,
            cursor="arrow", wrap=tk.NONE, highlightthickness=0)
        self._track_box.pack(fill=tk.X, padx=8)
        self._track_box.tag_configure("hdr", foreground=_FG,
                                      font=("Courier", 8, "bold"))
        self._track_box.tag_configure("val", foreground="#888888")
        self._track_box.tag_configure("dim", foreground="#444444")

        _sep(p)
        tk.Label(p, text="RADAR", bg=_PANEL, fg=_FG,
                 font=("Courier", 8), anchor="w").pack(fill=tk.X, padx=8)

        self._v_clat = tk.StringVar(value=str(self.c_lat))
        self._v_clon = tk.StringVar(value=str(self.c_lon))
        self._v_rng  = tk.StringVar(value=str(int(self.rng)))
        self._entry_row(p, "lat", self._v_clat)
        self._entry_row(p, "lon", self._v_clon)
        self._entry_row(p, "range nm", self._v_rng)
        tk.Button(p, text="Apply", command=self._apply,
                  bg=_BTN, fg=_FG, activebackground=_BTN_ACT,
                  font=("Courier", 8), relief=tk.FLAT, bd=0, cursor="hand2"
                  ).pack(fill=tk.X, padx=8, pady=4)

        _sep(p)
        self._v_status = tk.StringVar(value="—")
        tk.Label(p, textvariable=self._v_status, bg=_PANEL, fg=_FG_DIM,
                 font=("Courier", 7), justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=8)

    def _apply(self):
        try:
            self.c_lat = float(self._v_clat.get())
            self.c_lon = float(self._v_clon.get())
            self.rng   = float(self._v_rng.get())
        except ValueError:
            pass

    # ── main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        now = time.monotonic()
        dt  = now - self._tick
        self._tick = now
        self.sweep = (self.sweep + _SWEEP_SPD * dt) % 360.0

        with self._lock:
            for icao in list(self._illum):
                self._illum[icao] += dt

            for icao, ac in self._fleet.items():
                if ac.lat is None:
                    continue
                if icao not in self._history:
                    self._history[icao] = deque(maxlen=8)
                    self._illum[icao]   = 999.0
                hist = self._history[icao]
                if not hist or math.hypot(ac.lat - hist[-1][0],
                                          ac.lon - hist[-1][1]) > 1e-4:
                    hist.append((ac.lat, ac.lon))
                nm_e = ((ac.lon - self.c_lon) * 60.0
                        * math.cos(math.radians(self.c_lat)))
                nm_n = (ac.lat - self.c_lat) * 60.0
                brng = math.degrees(math.atan2(nm_e, nm_n)) % 360.0
                if (self.sweep - brng) % 360.0 <= _SWEEP_SPD * dt + 2.0:
                    self._illum[icao] = 0.0

            fleet   = dict(self._fleet)
            history = {k: list(v) for k, v in self._history.items()}
            illum   = dict(self._illum)

        self._draw(fleet, history, illum)
        self._update_panel(fleet, illum)
        self._v_status.set(self._rx_status)
        self.after(50, self._loop)

    # ── drawing ───────────────────────────────────────────────────────────────

    def _draw(self, fleet, history, illum):
        cv = self.cv
        cv.delete("all")
        cx, cy, r = self._geom()

        cv.create_oval(cx-r, cy-r, cx+r, cy+r, fill=_RADAR, outline="")

        step = 50 if self.rng <= 350 else 100
        ring = step
        while ring < self.rng:
            rp = int(r * ring / self.rng)
            cv.create_oval(cx-rp, cy-rp, cx+rp, cy+rp,
                           outline=_RING_D, width=1)
            a = math.radians(42)
            cv.create_text(cx + int(rp * math.sin(a)) + 2,
                           cy - int(rp * math.cos(a)) - 8,
                           text=str(ring), fill=_DIM, font=("Courier", 7))
            ring += step

        cv.create_oval(cx-r, cy-r, cx+r, cy+r, outline=_RING_B, width=1)
        cv.create_line(cx, cy-r, cx, cy+r, fill=_RING_D)
        cv.create_line(cx-r, cy, cx+r, cy, fill=_RING_D)

        for txt, dx, dy in (("N", 0, -(r+12)), ("S", 0, r+12),
                             ("W", -(r+13), 0), ("E", r+13, 0)):
            cv.create_text(cx+dx, cy+dy, text=txt,
                           fill=_RING_B, font=("Courier", 8))

        for off in range(-20, 1):
            ang = math.radians((self.sweep + off) % 360)
            cv.create_line(cx, cy,
                           cx + int(r * math.sin(ang)),
                           cy - int(r * math.cos(ang)),
                           fill=(_SWEEP if off == 0 else _SWEEP_T))

        cv.create_text(6, 6,
                       text=f"{self.c_lat:+.3f}  {self.c_lon:+.3f}  {self.rng:.0f}nm",
                       fill=_DIM, font=("Courier", 7), anchor="nw")

        # trails
        for icao, pts in history.items():
            if illum.get(icao, 999.0) > 30.0:
                continue
            for i, (la, lo) in enumerate(reversed(pts)):
                pt = self._to_xy(la, lo)
                if pt:
                    c = _TRAIL[min(i, len(_TRAIL) - 1)]
                    cv.create_oval(pt[0]-2, pt[1]-2, pt[0]+2, pt[1]+2,
                                   fill=c, outline="")

        # blips
        for icao, ac in fleet.items():
            if ac.lat is None:
                continue
            age = illum.get(icao, 999.0)
            if age > 30.0:
                continue
            pt = self._to_xy(ac.lat, ac.lon)
            if pt:
                self._blip(cv, pt[0], pt[1], ac, age)

    def _blip(self, cv, x, y, ac, age):
        col = _B_FRESH if age < 3 else (_B_MED if age < 10 else _B_OLD)
        hdg = math.radians(ac.track if ac.track is not None else
                           (ac.heading or 0.0))
        sz, v = 8, []
        for a in (hdg, hdg + math.radians(148), hdg - math.radians(148)):
            v += [x + sz * math.sin(a), y - sz * math.cos(a)]
        cv.create_polygon(v, fill=col, outline="")
        cs  = (ac.callsign or ac.icao).strip()
        alt = f"FL{ac.altitude//100:03d}" if ac.altitude else "???"
        cv.create_text(x+12, y-12, text=cs,
                       fill=_FG, font=("Courier", 8, "bold"), anchor="w")
        cv.create_text(x+12, y-2,  text=alt,
                       fill=_DIM, font=("Courier", 7), anchor="w")
        if ac.speed:
            cv.create_text(x+12, y+7, text=f"{ac.speed}kt",
                           fill=_DIM, font=("Courier", 7), anchor="w")

    def _update_panel(self, fleet, illum):
        active = [(icao, ac) for icao, ac in sorted(fleet.items())
                  if illum.get(icao, 999.0) <= 30.0 and ac.lat is not None]
        t = self._track_box
        t.config(state=tk.NORMAL)
        t.delete("1.0", tk.END)
        for icao, ac in active:
            cs  = (ac.callsign or "—").strip()
            alt = f"FL{ac.altitude//100:03d}" if ac.altitude else "—"
            spd = f"{ac.speed}kt" if ac.speed else "—"
            t.insert(tk.END, f" {icao}  {cs}\n", "hdr")
            t.insert(tk.END, f"  {alt}  {spd}\n", "val")
            t.insert(tk.END, f"  {ac.lat:+.4f}  {ac.lon:+.4f}\n\n", "dim")
        if not active:
            t.insert(tk.END, "  no tracks", "dim")
        t.config(state=tk.DISABLED)

    # ── RX thread ─────────────────────────────────────────────────────────────

    def _rx_loop(self, group, port, iface):
        """Decode UDP multicast into self._fleet. No tkinter calls here."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", port))
        mreq = struct.pack("4s4s",
                           socket.inet_aton(group), socket.inet_aton(iface))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(1.0)
        buf, n = "", 0
        try:
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                    buf += data.decode("ascii", errors="ignore")
                except socket.timeout:
                    continue
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        with self._lock:
                            decode_message(line, self._fleet)
                        n += 1
                self._rx_status = f"{group}:{port}  {n} msgs"
        finally:
            try:
                sock.setsockopt(socket.IPPROTO_IP,
                                socket.IP_DROP_MEMBERSHIP, mreq)
            except OSError:
                pass
            sock.close()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    cfg = net_config.load()
    p   = argparse.ArgumentParser(description="ADS-B Radar Display",
                                  formatter_class=argparse.RawDescriptionHelpFormatter,
                                  epilog=__doc__)
    p.add_argument("--group",  default=cfg["group"])
    p.add_argument("--port",   type=int, default=cfg["port"])
    p.add_argument("--iface",  default=cfg["iface"])
    p.add_argument("--centre", metavar="LAT,LON", default=None)
    p.add_argument("--range",  type=float, default=200.0)
    args = p.parse_args()
    lat, lon = 51.477, -0.461
    if args.centre:
        lat, lon = map(float, args.centre.split(","))
    App(args.group, args.port, args.iface, lat, lon, args.range).mainloop()


if __name__ == "__main__":
    main()
