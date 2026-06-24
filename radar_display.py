#!/usr/bin/env python3
"""
ADS-B Radar Display
===================
Live PPI radar receiver — rotating sweep, fading trails, track panel.

The window is resizable and the whole radar (disc, blips, labels, fonts)
scales with it.  F11 toggles fullscreen, Esc leaves it.

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
import radar_ui as ui
from adsb_decoder import Aircraft, decode_message


# ── Radar-specific palette ────────────────────────────────────────────────────

# Each target gets its own random colour (assigned on first sight) so crossing
# tracks stay distinguishable.  Recency is shown by fading that colour's
# brightness with age since the last message, rather than by changing hue.
def _age_fade(age):
    if age < 3:    return 1.00   # 0 – 3 s  : full brightness
    if age < 10:   return 0.70   # 3 – 10 s
    return 0.45                  # 10 – 30 s (older than 30 s is culled)

# Trail dots, newest to oldest
_TRAIL = ["#444444", "#3a3a3a", "#303030", "#262626",
          "#1e1e1e", "#181818", "#121212", "#0e0e0e"]


# ── App ───────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    """
    Radar display. RX thread decodes into self._fleet; main loop draws at 20 fps.
    All tkinter calls are main-thread only — thread writes only to self._fleet
    (under lock) and self._rx_status (plain string).
    """

    def __init__(self, group, port, iface, c_lat, c_lon, rng):
        super().__init__()
        self.title("Radar Display")
        self.configure(bg=ui.PANEL)
        self.resizable(True, True)
        self.minsize(round(420 * ui.SCALE), round(360 * ui.SCALE))

        self.c_lat, self.c_lon, self.rng = c_lat, c_lon, rng
        self._tick = time.monotonic()
        self._cw = self._ch = ui.CANVAS_SZ      # live canvas size (autoscale)
        self._fullscreen = False

        self._fleet:    dict[str, Aircraft] = {}
        self._history:  dict[str, deque]   = {}
        self._last_rx:  dict[str, float]   = {}   # monotonic time of last message
        self._colors:   dict[str, str]     = {}   # icao → random blip colour
        self._lock      = threading.Lock()
        self._rx_status = "joining…"
        self._bg_sig    = None    # view signature the cached rings were drawn for
        self._fg_sig    = None    # dynamic-state signature the fg layer was drawn for

        self._build_ui()
        self.bind("<F11>",    self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)
        threading.Thread(target=self._rx_loop,
                         args=(group, port, iface), daemon=True).start()
        self._loop()

    # ── coordinate helper ─────────────────────────────────────────────────────

    def _to_xy(self, lat, lon):
        cx, cy, r = ui.geom(self._cw, self._ch)
        return ui.ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    def _on_resize(self, ev):
        self._cw, self._ch = ev.width, ev.height

    def _toggle_fullscreen(self, _ev=None):
        self._fullscreen = not self._fullscreen
        self.attributes("-fullscreen", self._fullscreen)

    def _exit_fullscreen(self, _ev=None):
        self._fullscreen = False
        self.attributes("-fullscreen", False)

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=ui.CANVAS_SZ, height=ui.CANVAS_SZ,
                            bg=ui.BG, highlightthickness=0, cursor="none")
        self.cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.cv.bind("<Configure>", self._on_resize)

        p = ui.make_panel(self)

        tk.Frame(p, bg=ui.PANEL, height=round(10 * ui.SCALE)).pack()
        tk.Label(p, text="TRACKS", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)

        self._track_box = tk.Text(
            p, bg=ui.ENTRY, fg=ui.FG, font=ui.F_MD,
            relief=tk.FLAT, bd=0, height=14, state=tk.DISABLED,
            cursor="arrow", wrap=tk.NONE, highlightthickness=0)
        self._track_box.pack(fill=tk.X, padx=ui.PAD)
        self._track_box.tag_configure("hdr", foreground=ui.FG, font=ui.F_BLD)
        self._track_box.tag_configure("val", foreground="#888888")
        self._track_box.tag_configure("dim", foreground="#444444")

        ui.flat_button(p, "Clear screen", self._clear,
                       bg=ui.BTN_RED, fg="#ffffff", active=ui.BTN_RED_A
                       ).pack(fill=tk.X, padx=ui.PAD, pady=(ui.PAD2, 0))

        ui.sep(p)
        tk.Label(p, text="RADAR", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)

        self._v_clat = tk.StringVar(value=str(self.c_lat))
        self._v_clon = tk.StringVar(value=str(self.c_lon))
        self._v_rng  = tk.StringVar(value=str(int(self.rng)))
        ui.entry_row(p, "lat",      self._v_clat)
        ui.entry_row(p, "lon",      self._v_clon)
        ui.entry_row(p, "range nm", self._v_rng)
        ui.flat_button(p, "Apply", self._apply
                       ).pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD)

        ui.sep(p)
        self._v_status = tk.StringVar(value="—")
        tk.Label(p, textvariable=self._v_status, bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_SM, justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=ui.PAD)

    def _clear(self):
        """Drop all tracked aircraft, trails, and colours; the RX thread
        repopulates the screen as fresh messages arrive."""
        with self._lock:
            self._fleet.clear()
            self._history.clear()
            self._last_rx.clear()
            self._colors.clear()
        self._fg_sig = None        # force the now-empty fg layer to redraw once

    def _apply(self):
        try:
            self.c_lat = float(self._v_clat.get())
            self.c_lon = float(self._v_clon.get())
            self.rng   = float(self._v_rng.get())
        except ValueError:
            pass

    # ── main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        now        = time.monotonic()
        self._tick = now

        with self._lock:
            # Remove aircraft silent for >60 s
            gone = [icao for icao, t in self._last_rx.items()
                    if now - t > 60.0]
            for icao in gone:
                self._fleet.pop(icao, None)
                self._history.pop(icao, None)
                self._last_rx.pop(icao, None)
                self._colors.pop(icao, None)

            for icao, ac in self._fleet.items():
                if ac.lat is None:
                    continue
                if icao not in self._history:
                    self._history[icao] = deque(maxlen=8)
                hist = self._history[icao]
                if not hist or math.hypot(ac.lat - hist[-1][0],
                                          ac.lon - hist[-1][1]) > 1e-4:
                    hist.append((ac.lat, ac.lon))

            fleet   = dict(self._fleet)
            history = {k: list(v) for k, v in self._history.items()}
            # illum = seconds since last message received (matches display thresholds)
            illum   = {icao: now - t for icao, t in self._last_rx.items()}

        self._draw(fleet, history, illum)
        self._update_panel(fleet, illum)
        self._v_status.set(self._rx_status)
        self.after(50, self._loop)

    # ── drawing ───────────────────────────────────────────────────────────────

    def _draw(self, fleet, history, illum):
        cv = self.cv
        cx, cy, r = ui.geom(self._cw, self._ch)
        sf = ui.scale_for(self._cw, self._ch)

        # Static rings — cached; only redrawn when the view changes.
        sig = (round(self.c_lat, 6), round(self.c_lon, 6),
               round(self.rng, 3), self._cw, self._ch)
        if sig != self._bg_sig:
            cv.delete("bg")
            ui.draw_radar_frame(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                sf, tag="bg")
            self._bg_sig = sig
            self._fg_sig = None    # bg rebuild → force fg recreation

        # Dynamic trails + blips — only redraw when the visible state changes.
        # The age buckets (<3 s, <10 s, <30 s) only matter at their boundaries,
        # so quantising age to those buckets in the signature lets the cache
        # hold across sub-second-but-not-bucket-crossing frames.
        def bucket(age):
            return 0 if age < 3 else (1 if age < 10 else (2 if age < 30 else 3))
        fg_sig = (
            tuple((i, bucket(illum.get(i, 999.0)),
                   None if a.lat is None else round(a.lat, 5),
                   None if a.lon is None else round(a.lon, 5),
                   # None encodes "no heading yet" so the cache catches the
                   # None → 0.0 transition (true-North track is otherwise indistinct).
                   None if (a.track is None and a.heading is None)
                        else round((a.track if a.track is not None else a.heading), 1),
                   a.altitude, a.speed,
                   (a.callsign or "").strip())
                  for i, a in fleet.items()),
            tuple((i, tuple((round(la, 5), round(lo, 5)) for la, lo in pts))
                  for i, pts in history.items()),
            round(sf, 2),
        )
        if fg_sig == self._fg_sig:
            return
        self._fg_sig = fg_sig

        cv.delete("fg")
        td = ui.TRAIL_DOT * sf
        for icao, pts in history.items():
            if illum.get(icao, 999.0) > 30.0:
                continue
            for i, (la, lo) in enumerate(reversed(pts)):
                pt = self._to_xy(la, lo)
                if pt:
                    c = _TRAIL[min(i, len(_TRAIL) - 1)]
                    cv.create_oval(pt[0]-td, pt[1]-td, pt[0]+td, pt[1]+td,
                                   fill=c, outline="", tags="fg")

        for icao, ac in fleet.items():
            if ac.lat is None:
                continue
            age = illum.get(icao, 999.0)
            if age > 30.0:
                continue
            pt = self._to_xy(ac.lat, ac.lon)
            if pt:
                self._blip(cv, pt[0], pt[1], ac, age, sf)

    def _color(self, icao):
        """Return this target's persistent random colour, assigning one if new."""
        col = self._colors.get(icao)
        if col is None:
            col = self._colors[icao] = ui.random_color()
        return col

    def _blip(self, cv, x, y, ac, age, sf=1.0):
        col = ui.shade(self._color(ac.icao), _age_fade(age))
        hdg_deg = ac.track if ac.track is not None else ac.heading
        if hdg_deg is None:
            # Position received but no velocity message yet — render a circle
            # so we don't fake a heading (the old code defaulted to 0° = North).
            r = ui.BLIP_SZ * sf
            cv.create_oval(x-r, y-r, x+r, y+r, fill=col, outline="", tags="fg")
        else:
            ui.draw_blip(cv, x, y, math.radians(hdg_deg), col, sf, tag="fg")
        cs  = (ac.callsign or ac.icao).strip()
        alt = f"FL{ac.altitude//100:03d}" if ac.altitude else "???"
        dx, dy = ui.LBL_DX * sf, ui.LBL_DY * sf
        f_sm = ui.sfont(ui.PT_SM, sf)
        cv.create_text(x + dx, y - dy,
                       text=cs, fill=col, font=ui.sfont(ui.PT_MD, sf, bold=True),
                       anchor="w", tags="fg")
        cv.create_text(x + dx, y - round(2 * ui.SCALE * sf),
                       text=alt, fill=ui.DIM, font=f_sm, anchor="w", tags="fg")
        if ac.speed:
            cv.create_text(x + dx, y + round(7 * ui.SCALE * sf),
                           text=f"{ac.speed}kt", tags="fg",
                           fill=ui.DIM, font=f_sm, anchor="w")

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
                            # ICAO is hex chars 3-8 of a DF17 raw frame (*8DICAO...)
                            if len(line) >= 9:
                                self._last_rx[line[3:9].upper()] = time.monotonic()
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
