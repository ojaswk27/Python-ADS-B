#!/usr/bin/env python3
"""
ADS-B Radar Display
===================
Live PPI radar receiver. Joins the UDP multicast group, decodes incoming
ADS-B messages, and renders a sweep-radar with fading blip trails.

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


# ── Palette ───────────────────────────────────────────────────────────────────

_BG        = "#000000"
_RADAR_BG  = "#000a00"
_RING_D    = "#112211"
_RING_B    = "#00aa00"
_SWEEP_C   = "#00ff44"
_TRAIL_C   = "#003300"
_DIM       = "#336633"
_PANEL_BG  = "#0d0d0d"

# Blip colour by age since last sweep illumination
_BLIP_FRESH = "#00ff00"   # 0 – 3 s
_BLIP_MED   = "#008800"   # 3 – 10 s
_BLIP_OLD   = "#004400"   # 10 – 30 s

# Trail dot colours: index 0 = most recent, 7 = oldest
_TRAIL = ["#005500", "#004a00", "#004000", "#003600",
          "#002c00", "#002200", "#001800", "#001000"]

_CANVAS = 680
_PANEL  = 240
_SWEEP  = 36.0   # degrees per second — one rotation every 10 s


# ── Coordinate helper ─────────────────────────────────────────────────────────

def _ll_to_xy(lat, lon, cx, cy, r_px, c_lat, c_lon, range_nm):
    scale = r_px / range_nm
    nm_e  = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    nm_n  = (lat - c_lat) * 60.0
    if math.hypot(nm_e, nm_n) > range_nm * 1.02:
        return None
    return cx + nm_e * scale, cy - nm_n * scale


# ── Application ───────────────────────────────────────────────────────────────

class App(tk.Tk):
    """
    Radar display application.

    A background RX thread receives and decodes UDP multicast ADS-B messages
    into a shared fleet dict.  The main loop animates the rotating sweep, draws
    fading position trails for each track, and refreshes the side panel.

    All tkinter calls happen exclusively on the main thread.  The RX thread
    writes only to self._fleet (protected by self._lock) and self._rx_status
    (a plain string read by the main loop).
    """

    def __init__(self, group, port, iface, c_lat, c_lon, range_nm):
        super().__init__()
        self.title("ADS-B Radar Display")
        self.configure(bg=_PANEL_BG)
        self.resizable(False, False)

        self.c_lat    = c_lat
        self.c_lon    = c_lon
        self.range_nm = range_nm
        self.sweep    = 0.0
        self._tick    = time.monotonic()

        self._fleet:   dict[str, Aircraft] = {}
        self._history: dict[str, deque]    = {}   # icao → deque[(lat, lon)]
        self._illum:   dict[str, float]    = {}   # icao → seconds since illuminated
        self._lock     = threading.Lock()
        self._rx_status = "joining…"

        self._build_ui()
        threading.Thread(target=self._rx_loop,
                         args=(group, port, iface), daemon=True).start()
        self._loop()

    # ── Geometry ──────────────────────────────────────────────────────────────

    def _geom(self):
        cx = cy = _CANVAS // 2
        return cx, cy, cx - 18

    def _to_xy(self, lat, lon):
        cx, cy, r = self._geom()
        return _ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.range_nm)

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=_CANVAS, height=_CANVAS,
                            bg=_BG, highlightthickness=0, cursor="none")
        self.cv.pack(side=tk.LEFT, padx=4, pady=4)

        pf = tk.Frame(self, bg=_PANEL_BG, width=_PANEL)
        pf.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4), pady=4)
        pf.pack_propagate(False)
        self._build_panel(pf)

    def _sec(self, parent, title):
        f = tk.LabelFrame(parent, text=f" {title} ", bg=_PANEL_BG, fg="#00cc00",
                          font=("Courier", 9, "bold"), relief=tk.GROOVE, bd=1)
        f.pack(fill=tk.X, padx=4, pady=(6, 0))
        return f

    def _field(self, parent, label, var):
        row = tk.Frame(parent, bg=_PANEL_BG)
        row.pack(fill=tk.X, padx=4, pady=1)
        tk.Label(row, text=label, bg=_PANEL_BG, fg="#668866",
                 font=("Courier", 8), width=10, anchor="w").pack(side=tk.LEFT)
        tk.Entry(row, textvariable=var, width=9,
                 bg="#0d1a0d", fg="#00ff00", insertbackground="#00ff00",
                 font=("Courier", 9), relief=tk.FLAT
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True)

    def _build_panel(self, pf):
        # Track list (read-only text widget for flexible formatting)
        tf = self._sec(pf, "Tracks")
        self._track_box = tk.Text(
            tf, bg="#050f05", fg="#00ff00", font=("Courier", 8),
            relief=tk.FLAT, bd=0, height=14, state=tk.DISABLED,
            cursor="arrow", wrap=tk.NONE)
        self._track_box.pack(fill=tk.X, padx=4, pady=(4, 4))
        self._track_box.tag_configure("hdr",  foreground="#00cc00",
                                      font=("Courier", 8, "bold"))
        self._track_box.tag_configure("data", foreground="#00ff88")
        self._track_box.tag_configure("dim",  foreground="#336633")

        # Radar settings
        rf = self._sec(pf, "Radar")
        self._v_clat = tk.StringVar(value=str(self.c_lat))
        self._v_clon = tk.StringVar(value=str(self.c_lon))
        self._v_rnm  = tk.StringVar(value=str(int(self.range_nm)))
        self._field(rf, "Centre lat", self._v_clat)
        self._field(rf, "Centre lon", self._v_clon)
        self._field(rf, "Range nm",   self._v_rnm)
        tk.Button(rf, text="Apply", command=self._apply_radar,
                  bg=_PANEL_BG, fg="#00cc00", activebackground="#1a331a",
                  font=("Courier", 8, "bold"), relief=tk.FLAT, cursor="hand2"
                  ).pack(fill=tk.X, padx=4, pady=4)

        # Receive status
        sf = self._sec(pf, "Receive")
        self._v_status = tk.StringVar(value="joining…")
        tk.Label(sf, textvariable=self._v_status, bg=_PANEL_BG, fg="#668866",
                 font=("Courier", 7), justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=4, pady=4)

    def _apply_radar(self):
        try:
            self.c_lat    = float(self._v_clat.get())
            self.c_lon    = float(self._v_clon.get())
            self.range_nm = float(self._v_rnm.get())
        except ValueError:
            pass

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        now = time.monotonic()
        dt  = now - self._tick
        self._tick = now
        self.sweep = (self.sweep + _SWEEP * dt) % 360.0

        with self._lock:
            # Age all illumination timers
            for icao in list(self._illum):
                self._illum[icao] += dt

            for icao, ac in self._fleet.items():
                if ac.lat is None:
                    continue

                # Initialise history and illumination for new tracks
                if icao not in self._history:
                    self._history[icao] = deque(maxlen=8)
                    self._illum[icao]   = 999.0

                # Record position when it changes noticeably
                hist = self._history[icao]
                if not hist or math.hypot(ac.lat - hist[-1][0],
                                          ac.lon - hist[-1][1]) > 1e-4:
                    hist.append((ac.lat, ac.lon))

                # Illuminate when the sweep line sweeps over this track
                nm_e = (ac.lon - self.c_lon) * 60.0 * math.cos(math.radians(self.c_lat))
                nm_n = (ac.lat - self.c_lat) * 60.0
                bearing = math.degrees(math.atan2(nm_e, nm_n)) % 360.0
                if (self.sweep - bearing) % 360.0 <= _SWEEP * dt + 2.0:
                    self._illum[icao] = 0.0

            fleet_snap   = dict(self._fleet)
            history_snap = {k: list(v) for k, v in self._history.items()}
            illum_snap   = dict(self._illum)

        self._draw(fleet_snap, history_snap, illum_snap)
        self._update_panel(fleet_snap, illum_snap)
        self._v_status.set(self._rx_status)
        self.after(50, self._loop)

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _draw(self, fleet, history, illum):
        cv = self.cv
        cv.delete("all")
        cx, cy, r = self._geom()

        # Radar disc
        cv.create_oval(cx-r, cy-r, cx+r, cy+r, fill=_RADAR_BG, outline="")

        # Range rings
        step = 50 if self.range_nm <= 350 else 100
        ring = step
        while ring < self.range_nm:
            rp = int(r * ring / self.range_nm)
            cv.create_oval(cx-rp, cy-rp, cx+rp, cy+rp,
                           outline=_RING_D, width=1, dash=(3, 6))
            a = math.radians(42)
            cv.create_text(cx + int(rp * math.sin(a)) + 2,
                           cy - int(rp * math.cos(a)) - 8,
                           text=str(ring), fill=_DIM, font=("Courier", 7))
            ring += step

        cv.create_oval(cx-r, cy-r, cx+r, cy+r, outline=_RING_B, width=2)
        cv.create_line(cx, cy-r, cx, cy+r, fill=_RING_D, dash=(4, 8))
        cv.create_line(cx-r, cy, cx+r, cy, fill=_RING_D, dash=(4, 8))

        for txt, dx, dy in (("N", 0, -(r+13)), ("S", 0, r+13),
                             ("W", -(r+13), 0), ("E", r+13, 0)):
            cv.create_text(cx+dx, cy+dy, text=txt,
                           fill=_RING_B, font=("Courier", 9, "bold"))

        # Sweep line with 20° fading trail
        for off in range(-20, 1):
            ang = math.radians((self.sweep + off) % 360)
            ex  = cx + int(r * math.sin(ang))
            ey  = cy - int(r * math.cos(ang))
            cv.create_line(cx, cy, ex, ey,
                           fill=(_SWEEP_C if off == 0 else _TRAIL_C),
                           width=(2 if off == 0 else 1))

        cv.create_oval(cx-3, cy-3, cx+3, cy+3, fill=_RING_B, outline="")
        cv.create_text(8, 8,
                       text=f"⊕ {self.c_lat:+.3f}°  {self.c_lon:+.3f}°   {self.range_nm:.0f} nm",
                       fill=_DIM, font=("Courier", 7), anchor="nw")

        # Position trails (draw before blips so blips sit on top)
        for icao, pts in history.items():
            if illum.get(icao, 999.0) > 30.0:
                continue
            for i, (lat, lon) in enumerate(reversed(pts)):
                pt = self._to_xy(lat, lon)
                if pt is None:
                    continue
                color = _TRAIL[min(i, len(_TRAIL) - 1)]
                cv.create_oval(pt[0]-2, pt[1]-2, pt[0]+2, pt[1]+2,
                               fill=color, outline="")

        # Blips
        for icao, ac in fleet.items():
            if ac.lat is None:
                continue
            age = illum.get(icao, 999.0)
            if age > 30.0:
                continue
            pt = self._to_xy(ac.lat, ac.lon)
            if pt is None:
                continue
            self._draw_blip(cv, pt[0], pt[1], ac, age)

    def _draw_blip(self, cv, x, y, ac, age):
        color = _BLIP_FRESH if age < 3.0 else (_BLIP_MED if age < 10.0 else _BLIP_OLD)

        hdg = math.radians(
            ac.track if ac.track is not None else
            (ac.heading if ac.heading is not None else 0.0))
        sz    = 8
        verts = []
        for a in (hdg, hdg + math.radians(148), hdg - math.radians(148)):
            verts += [x + sz * math.sin(a), y - sz * math.cos(a)]
        cv.create_polygon(verts, fill=color, outline=color)

        cs  = (ac.callsign or ac.icao).strip()
        alt = f"FL{ac.altitude//100:03d}" if ac.altitude else "???"
        cv.create_text(x+13, y-14, text=cs,  fill="#aaffaa",
                       font=("Courier", 8, "bold"), anchor="w")
        cv.create_text(x+13, y-4,  text=alt, fill=_DIM,
                       font=("Courier", 7), anchor="w")
        if ac.speed:
            cv.create_text(x+13, y+5, text=f"{ac.speed}kt", fill=_DIM,
                           font=("Courier", 7), anchor="w")

    def _update_panel(self, fleet, illum):
        """Rebuild the track list in the side panel (main thread only)."""
        active = [(icao, ac) for icao, ac in sorted(fleet.items())
                  if illum.get(icao, 999.0) <= 30.0 and ac.lat is not None]

        t = self._track_box
        t.config(state=tk.NORMAL)
        t.delete("1.0", tk.END)

        for icao, ac in active:
            cs  = (ac.callsign or "—").strip()
            alt = f"FL{ac.altitude//100:03d}" if ac.altitude else "—"
            spd = f"{ac.speed}kt" if ac.speed else "—"
            trk = (f"{ac.track:.0f}°"   if ac.track   is not None else
                   f"{ac.heading:.0f}°" if ac.heading is not None else "—")
            t.insert(tk.END, f" {icao}\n", "hdr")
            t.insert(tk.END, f"  {cs:<9}{alt:<8}{spd}\n", "data")
            t.insert(tk.END, f"  {ac.lat:+.4f}°  {ac.lon:+.4f}°\n", "dim")
            t.insert(tk.END, f"  trk {trk}\n\n", "dim")

        if not active:
            t.insert(tk.END, "  no tracks\n", "dim")
        t.insert(tk.END, f"\n  {len(active)} active", "dim")
        t.config(state=tk.DISABLED)

    # ── RX thread ─────────────────────────────────────────────────────────────

    def _rx_loop(self, group, port, iface):
        """
        Receive UDP multicast datagrams and decode into self._fleet.
        No tkinter calls — status is written to self._rx_status (plain string).
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", port))
        mreq = struct.pack("4s4s",
                           socket.inet_aton(group), socket.inet_aton(iface))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(1.0)

        buf       = ""
        msg_count = 0
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
                        msg_count += 1
                self._rx_status = (f"{group}:{port}\n"
                                   f"{msg_count} msgs received")
        finally:
            try:
                sock.setsockopt(socket.IPPROTO_IP,
                                socket.IP_DROP_MEMBERSHIP, mreq)
            except OSError:
                pass
            sock.close()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    _cfg = net_config.load()
    parser = argparse.ArgumentParser(
        description="ADS-B Radar Display — live PPI receiver",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--group",  default=_cfg["group"])
    parser.add_argument("--port",   type=int, default=_cfg["port"])
    parser.add_argument("--iface",  default=_cfg["iface"])
    parser.add_argument("--centre", metavar="LAT,LON", default=None,
                        help="Radar centre lat,lon (default: 51.477,-0.461)")
    parser.add_argument("--range",  type=float, default=200.0,
                        help="Display range in nm (default: 200)")
    args = parser.parse_args()

    lat, lon = 51.477, -0.461
    if args.centre:
        lat, lon = map(float, args.centre.split(","))

    App(args.group, args.port, args.iface, lat, lon, args.range).mainloop()


if __name__ == "__main__":
    main()
