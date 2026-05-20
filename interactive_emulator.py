#!/usr/bin/env python3
"""
ADS-B Interactive Emulator
==========================
Click anywhere inside the PPI circle to place a simulated aircraft.
The click coordinates are converted to lat/lon and ADS-B messages are
transmitted immediately over UDP multicast.

Controls
--------
    Left-click empty space  — place new aircraft
    Left-click + drag       — move an existing aircraft
    Right-click blip        — delete that aircraft
    Edit panel → Update     — change altitude / speed / heading
    Radar panel → Apply     — recentre / rescale the display

Usage
-----
    python interactive_emulator.py
    python interactive_emulator.py --group 239.255.0.1 --port 30003 --iface 127.0.0.1
    python interactive_emulator.py --centre 51.5,-0.5 --range 150
"""

import argparse
import math
import socket
import threading
import time
import tkinter as tk

from aircraft_emulator import build_identification, build_position, build_velocity


# ═══════════════════════════════════════════════════════════════════════════════
# Section 1 — Simulated aircraft model
# ═══════════════════════════════════════════════════════════════════════════════

_counter = [0]


def _next_icao() -> str:
    _counter[0] += 1
    return f"FF{_counter[0]:04X}"


def _next_callsign() -> str:
    return f"SIM{_counter[0]:03d}"


class ClickAircraft:
    """
    A single user-placed simulated aircraft.
    Position updates continuously when speed > 0.
    Messages are emitted in round-robin: ident → even → odd → vel (repeat).
    """

    __slots__ = ("icao", "callsign", "lat", "lon",
                 "alt_ft", "speed_kt", "heading_deg", "_msg_idx")

    def __init__(self, lat: float, lon: float,
                 alt_ft: int = 35_000,
                 speed_kt: int = 0,
                 heading_deg: float = 0.0) -> None:
        self.icao        = _next_icao()
        self.callsign    = _next_callsign()
        self.lat         = lat
        self.lon         = lon
        self.alt_ft      = alt_ft
        self.speed_kt    = speed_kt
        self.heading_deg = heading_deg
        self._msg_idx    = 0

    def step(self, dt: float) -> None:
        """Advance position by dt seconds at current speed/heading."""
        if self.speed_kt == 0:
            return
        hdg = math.radians(self.heading_deg)
        nm  = self.speed_kt * dt / 3600.0
        self.lat += nm * math.cos(hdg) / 60.0
        self.lon += nm * math.sin(hdg) / (60.0 * math.cos(math.radians(self.lat)))

    def next_message(self) -> tuple:
        """Return (label, hex_string) for the next message in the cycle."""
        seq = self._msg_idx % 7
        self._msg_idx += 1
        if seq == 0:
            return "IDENT", build_identification(self.icao, self.callsign)
        if seq in (1, 4):
            return "POS-E", build_position(
                self.icao, self.lat, self.lon, self.alt_ft, False)
        if seq in (2, 5):
            return "POS-O", build_position(
                self.icao, self.lat, self.lon, self.alt_ft, True)
        return "VEL", build_velocity(
            self.icao, self.speed_kt, self.heading_deg, 0)


# ═══════════════════════════════════════════════════════════════════════════════
# Section 2 — Coordinate transforms (pixel ↔ lat/lon)
# ═══════════════════════════════════════════════════════════════════════════════

def _ll_to_xy(lat, lon, cx, cy, r_px, c_lat, c_lon, range_nm):
    """Lat/lon → canvas (x, y).  Returns None if outside the radar circle."""
    scale = r_px / range_nm
    nm_e  = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    nm_n  = (lat - c_lat) * 60.0
    if math.hypot(nm_e, nm_n) > range_nm * 1.02:
        return None
    return cx + nm_e * scale, cy - nm_n * scale


def _xy_to_ll(x, y, cx, cy, r_px, c_lat, c_lon, range_nm):
    """Canvas (x, y) → (lat, lon).  Returns None if outside the radar circle."""
    scale = r_px / range_nm
    nm_e  = (x - cx) / scale
    nm_n  = (cy - y) / scale
    if math.hypot(nm_e, nm_n) > range_nm:
        return None
    lat = c_lat + nm_n / 60.0
    lon = c_lon + nm_e / (60.0 * math.cos(math.radians(c_lat)))
    return lat, lon


# ═══════════════════════════════════════════════════════════════════════════════
# Section 3 — Application
# ═══════════════════════════════════════════════════════════════════════════════

_CANVAS  = 680          # square canvas side in pixels
_PANEL   = 230          # right panel width
_HIT     = 14           # click-selection radius in pixels
_SWEEP   = 36.0         # degrees per second (1 rotation / 10 s)

# Radar colour palette
_BG      = "#000000"
_RING_D  = "#112211"    # dim ring
_RING_B  = "#00aa00"    # bright ring / compass
_SWEEP_C = "#00ff44"    # sweep tip
_TRAIL_C = "#003300"    # sweep trail
_BLIP    = "#00ff00"    # aircraft blip
_SEL     = "#ffff00"    # selected blip
_LBL     = "#aaffaa"    # callsign label
_DIM     = "#336633"    # range labels / vector


class App(tk.Tk):

    def __init__(self, group: str, port: int, iface: str,
                 centre_lat: float, centre_lon: float, range_nm: float) -> None:
        super().__init__()
        self.title("ADS-B Interactive Emulator")
        self.configure(bg="#111111")
        self.resizable(False, False)

        # Radar state
        self.c_lat    = centre_lat
        self.c_lon    = centre_lon
        self.range_nm = range_nm
        self.sweep    = 0.0
        self._tick    = time.monotonic()

        # Transmit
        self.group = group
        self.port  = port
        self._sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                              socket.inet_aton(iface))

        # Aircraft
        self._aircraft: list[ClickAircraft] = []
        self._selected: ClickAircraft | None = None
        self._drag_ac:  ClickAircraft | None = None
        self._lock = threading.Lock()
        self._tx_count = 0

        self._build_ui()
        threading.Thread(target=self._tx_loop, daemon=True).start()
        self._loop()

    # ── PPI geometry helpers ──────────────────────────────────────────────────

    def _geom(self):
        cx = cy = _CANVAS // 2
        return cx, cy, cx - 18   # (cx, cy, r_px)

    def _to_xy(self, lat, lon):
        cx, cy, r = self._geom()
        return _ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.range_nm)

    def _to_ll(self, x, y):
        cx, cy, r = self._geom()
        return _xy_to_ll(x, y, cx, cy, r, self.c_lat, self.c_lon, self.range_nm)

    def _nearest(self, x, y):
        best, best_d = None, _HIT
        for ac in self._aircraft:
            pt = self._to_xy(ac.lat, ac.lon)
            if pt is None:
                continue
            d = math.hypot(x - pt[0], y - pt[1])
            if d < best_d:
                best, best_d = ac, d
        return best

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=_CANVAS, height=_CANVAS,
                            bg=_BG, cursor="crosshair", highlightthickness=0)
        self.cv.pack(side=tk.LEFT, padx=4, pady=4)
        self.cv.bind("<Button-1>",        self._press)
        self.cv.bind("<B1-Motion>",       self._drag)
        self.cv.bind("<ButtonRelease-1>", self._release)
        self.cv.bind("<Motion>",          self._hover)
        self.cv.bind("<Button-2>",        self._right)
        self.cv.bind("<Button-3>",        self._right)

        pf = tk.Frame(self, bg="#111111", width=_PANEL)
        pf.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4), pady=4)
        pf.pack_propagate(False)
        self._panel(pf)

    def _sec(self, p, title):
        f = tk.LabelFrame(p, text=f" {title} ", bg="#111111", fg="#00cc00",
                          font=("Courier", 9, "bold"), relief=tk.GROOVE, bd=1)
        f.pack(fill=tk.X, padx=4, pady=(6, 0))
        return f

    def _field(self, parent, label, var, w=9):
        row = tk.Frame(parent, bg="#111111")
        row.pack(fill=tk.X, padx=4, pady=1)
        tk.Label(row, text=label, bg="#111111", fg="#668866",
                 font=("Courier", 8), width=11, anchor="w").pack(side=tk.LEFT)
        tk.Entry(row, textvariable=var, width=w,
                 bg="#0d1a0d", fg="#00ff00", insertbackground="#00ff00",
                 font=("Courier", 9), relief=tk.FLAT, bd=1
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True)

    def _btn(self, parent, text, cmd, fg="#00cc00"):
        return tk.Button(parent, text=text, command=cmd,
                         bg="#0d1a0d", fg=fg, activebackground="#1a331a",
                         activeforeground=fg, font=("Courier", 8, "bold"),
                         relief=tk.FLAT, bd=1, cursor="hand2")

    def _panel(self, pf):
        # Radar
        rf = self._sec(pf, "Radar")
        self._v_clat = tk.StringVar(value=str(self.c_lat))
        self._v_clon = tk.StringVar(value=str(self.c_lon))
        self._v_rnm  = tk.StringVar(value=str(int(self.range_nm)))
        self._field(rf, "Centre lat",  self._v_clat)
        self._field(rf, "Centre lon",  self._v_clon)
        self._field(rf, "Range (nm)",  self._v_rnm)
        self._btn(rf, "Apply", self._apply_radar).pack(
            fill=tk.X, padx=4, pady=4)

        # Defaults for new aircraft
        nf = self._sec(pf, "New Aircraft")
        self._v_dalt = tk.StringVar(value="35000")
        self._v_dspd = tk.StringVar(value="0")
        self._v_dhdg = tk.StringVar(value="0")
        self._field(nf, "Altitude ft", self._v_dalt)
        self._field(nf, "Speed kt",    self._v_dspd)
        self._field(nf, "Heading °",   self._v_dhdg)
        tk.Label(nf, text="  Left-click PPI to place",
                 bg="#111111", fg="#336633", font=("Courier", 7)
                 ).pack(anchor="w", padx=4, pady=(0, 4))

        # Track list
        tf = self._sec(pf, "Tracks")
        self._lb = tk.Listbox(
            tf, bg="#050f05", fg="#00ff00", selectbackground="#003300",
            selectforeground="#00ff88", font=("Courier", 8),
            relief=tk.FLAT, bd=0, height=6, activestyle="none")
        self._lb.pack(fill=tk.X, padx=4, pady=(4, 0))
        self._lb.bind("<<ListboxSelect>>", self._lb_select)
        bf = tk.Frame(tf, bg="#111111")
        bf.pack(fill=tk.X, padx=4, pady=4)
        self._btn(bf, "Delete", self._delete, "#cc4400"
                  ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        self._btn(bf, "Clear all", self._clear_all, "#cc4400"
                  ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Edit selected
        ef = self._sec(pf, "Edit Selected")
        self._v_eid  = tk.StringVar(value="—")
        self._v_elat = tk.StringVar()
        self._v_elon = tk.StringVar()
        self._v_ealt = tk.StringVar()
        self._v_espd = tk.StringVar()
        self._v_ehdg = tk.StringVar()
        tk.Label(ef, textvariable=self._v_eid, bg="#111111", fg="#00cc00",
                 font=("Courier", 8, "bold")).pack(pady=(4, 0))
        self._field(ef, "Lat",      self._v_elat)
        self._field(ef, "Lon",      self._v_elon)
        self._field(ef, "Alt ft",   self._v_ealt)
        self._field(ef, "Speed kt", self._v_espd)
        self._field(ef, "Hdg °",    self._v_ehdg)
        self._btn(ef, "Update", self._update_sel
                  ).pack(fill=tk.X, padx=4, pady=4)

        # Status
        sf = self._sec(pf, "Transmit")
        self._v_status = tk.StringVar(value="waiting…")
        tk.Label(sf, textvariable=self._v_status, bg="#111111", fg="#668866",
                 font=("Courier", 7), justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=4, pady=4)

        # Cursor lat/lon readout
        self._v_cursor = tk.StringVar(value="")
        tk.Label(pf, textvariable=self._v_cursor, bg="#111111", fg="#336633",
                 font=("Courier", 8), justify=tk.LEFT, anchor="w"
                 ).pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=6)

    # ── Mouse events ──────────────────────────────────────────────────────────

    def _press(self, ev):
        hit = self._nearest(ev.x, ev.y)
        if hit:
            self._drag_ac = hit
            self._select(hit)
        else:
            ll = self._to_ll(ev.x, ev.y)
            if ll:
                self._place(*ll)

    def _drag(self, ev):
        if self._drag_ac is None:
            return
        ll = self._to_ll(ev.x, ev.y)
        if ll:
            with self._lock:
                self._drag_ac.lat = ll[0]
                self._drag_ac.lon = ll[1]
            self._refresh_editor()

    def _release(self, _ev):
        self._drag_ac = None

    def _hover(self, ev):
        ll = self._to_ll(ev.x, ev.y)
        if ll:
            self._v_cursor.set(f"lat  {ll[0]:+.5f}°\nlon  {ll[1]:+.5f}°")
        else:
            self._v_cursor.set("")

    def _right(self, ev):
        hit = self._nearest(ev.x, ev.y)
        if hit:
            self._select(hit)
            self._delete()

    # ── Aircraft management ───────────────────────────────────────────────────

    def _place(self, lat, lon):
        try:
            alt = int(self._v_dalt.get() or 35000)
            spd = int(self._v_dspd.get() or 0)
            hdg = float(self._v_dhdg.get() or 0)
        except ValueError:
            alt, spd, hdg = 35_000, 0, 0.0
        ac = ClickAircraft(lat, lon, alt, spd, hdg)
        with self._lock:
            self._aircraft.append(ac)
        self._select(ac)
        self._refresh_list()

    def _select(self, ac):
        self._selected = ac
        self._refresh_editor()
        idx = next((i for i, a in enumerate(self._aircraft) if a is ac), None)
        if idx is not None:
            self._lb.selection_clear(0, tk.END)
            self._lb.selection_set(idx)
            self._lb.see(idx)

    def _lb_select(self, _ev):
        sel = self._lb.curselection()
        if not sel:
            return
        idx = sel[0]
        if idx < len(self._aircraft):
            ac = self._aircraft[idx]
            if ac is not self._selected:
                self._select(ac)

    def _refresh_list(self):
        self._lb.delete(0, tk.END)
        sel_idx = None
        for i, ac in enumerate(self._aircraft):
            tag = f" {ac.speed_kt}kt" if ac.speed_kt else ""
            self._lb.insert(tk.END, f" {ac.icao}  {ac.callsign}{tag}")
            if ac is self._selected:
                sel_idx = i
        if sel_idx is not None:
            self._lb.selection_set(sel_idx)

    def _refresh_editor(self):
        ac = self._selected
        if ac is None:
            self._v_eid.set("—")
            return
        self._v_eid.set(f"{ac.icao}  {ac.callsign}")
        self._v_elat.set(f"{ac.lat:+.5f}")
        self._v_elon.set(f"{ac.lon:+.5f}")
        self._v_ealt.set(str(ac.alt_ft))
        self._v_espd.set(str(ac.speed_kt))
        self._v_ehdg.set(f"{ac.heading_deg:.1f}")

    def _delete(self):
        if self._selected is None:
            return
        with self._lock:
            try:
                self._aircraft.remove(self._selected)
            except ValueError:
                pass
        self._selected = None
        self._v_eid.set("—")
        self._refresh_list()

    def _clear_all(self):
        with self._lock:
            self._aircraft.clear()
        self._selected = None
        self._v_eid.set("—")
        self._lb.delete(0, tk.END)

    def _update_sel(self):
        ac = self._selected
        if ac is None:
            return
        try:
            with self._lock:
                ac.lat         = float(self._v_elat.get())
                ac.lon         = float(self._v_elon.get())
                ac.alt_ft      = int(self._v_ealt.get())
                ac.speed_kt    = int(self._v_espd.get())
                ac.heading_deg = float(self._v_ehdg.get())
        except ValueError:
            pass
        self._refresh_list()

    def _apply_radar(self):
        try:
            self.c_lat    = float(self._v_clat.get())
            self.c_lon    = float(self._v_clon.get())
            self.range_nm = float(self._v_rnm.get())
        except ValueError:
            pass

    # ── Draw loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        now  = time.monotonic()
        dt   = now - self._tick
        self._tick = now

        self.sweep = (self.sweep + _SWEEP * dt) % 360.0

        with self._lock:
            for ac in self._aircraft:
                ac.step(dt)
            if self._selected:
                self._refresh_editor()

        self._draw()
        self.after(60, self._loop)   # ~16 fps

    def _draw(self):
        cv = self.cv
        cv.delete("all")
        cx, cy, r = self._geom()

        # Dark green fill inside the radar circle
        cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                       fill="#000a00", outline="")

        # Range rings
        step = 50 if self.range_nm <= 350 else 100
        ring = step
        while ring < self.range_nm:
            rp = int(r * ring / self.range_nm)
            cv.create_oval(cx - rp, cy - rp, cx + rp, cy + rp,
                           outline=_RING_D, width=1, dash=(3, 6))
            ang = math.radians(42)
            cv.create_text(cx + int(rp * math.sin(ang)) + 2,
                           cy - int(rp * math.cos(ang)) - 8,
                           text=str(ring), fill=_DIM, font=("Courier", 7))
            ring += step

        # Outer circle
        cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                       outline=_RING_B, width=2)

        # Cross-hairs
        cv.create_line(cx, cy - r, cx, cy + r, fill=_RING_D)
        cv.create_line(cx - r, cy, cx + r, cy, fill=_RING_D)

        # Cardinal labels
        for txt, dx, dy in (("N", 0, -r - 13), ("S", 0, r + 13),
                             ("W", -r - 13, 0), ("E", r + 13, 0)):
            cv.create_text(cx + dx, cy + dy, text=txt,
                           fill=_RING_B, font=("Courier", 10, "bold"))

        # Sweep — tip + 20° fading trail
        for off in range(-20, 1):
            ang = math.radians((self.sweep + off) % 360)
            ex  = cx + int(r * math.sin(ang))
            ey  = cy - int(r * math.cos(ang))
            col = _SWEEP_C if off == 0 else _TRAIL_C
            cv.create_line(cx, cy, ex, ey,
                           fill=col, width=(2 if off == 0 else 1))

        # Centre dot
        cv.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                       fill=_RING_B, outline="")

        # Centre label (top-left of canvas)
        cv.create_text(8, 8,
                       text=(f"⊕ {self.c_lat:+.3f}°  {self.c_lon:+.3f}°"
                             f"   {self.range_nm:.0f} nm"),
                       fill=_DIM, font=("Courier", 7), anchor="nw")

        # Aircraft blips
        with self._lock:
            acs = list(self._aircraft)
        for ac in acs:
            pt = self._to_xy(ac.lat, ac.lon)
            if pt is None:
                continue
            self._draw_blip(cv, cx, cy, r, pt[0], pt[1], ac)

    def _draw_blip(self, cv, cx, cy, r, x, y, ac):
        sel  = ac is self._selected
        fill = _SEL  if sel else _BLIP
        out  = "#ffffff" if sel else _BLIP
        hdg  = math.radians(ac.heading_deg)
        sz   = 9

        # Filled triangle pointing in heading direction
        pts = []
        for a in (hdg, hdg + math.radians(148), hdg - math.radians(148)):
            pts += [x + sz * math.sin(a), y - sz * math.cos(a)]
        cv.create_polygon(pts, fill=fill, outline=out, width=1)

        # Labels
        lx, ly = x + 14, y - 16
        cv.create_text(lx, ly,      text=ac.callsign,
                       fill=_LBL, font=("Courier", 8, "bold"), anchor="w")
        cv.create_text(lx, ly + 10, text=f"FL{ac.alt_ft // 100:03d}",
                       fill=_DIM, font=("Courier", 7), anchor="w")
        if ac.speed_kt:
            cv.create_text(lx, ly + 19, text=f"{ac.speed_kt} kt",
                           fill=_DIM, font=("Courier", 7), anchor="w")

        # 60-second velocity vector
        if ac.speed_kt > 0:
            scale = r / self.range_nm
            nm60  = ac.speed_kt * 60 / 3600.0
            vx    = x + nm60 * scale * math.sin(hdg)
            vy    = y - nm60 * scale * math.cos(hdg)
            cv.create_line(x, y, vx, vy, fill=_DIM, width=1, dash=(4, 3))

    # ── Transmit thread ───────────────────────────────────────────────────────

    def _tx_loop(self):
        while True:
            with self._lock:
                acs = list(self._aircraft)
            if not acs:
                time.sleep(0.1)
                continue
            for ac in acs:
                _, raw = ac.next_message()
                try:
                    self._sock.sendto(
                        f"*{raw};\n".encode(), (self.group, self.port))
                    self._tx_count += 1
                except OSError:
                    pass
                time.sleep(0.08)   # 12.5 msgs/s total
            self.after(0, self._v_status.set,
                       f"{self.group}:{self.port}\n"
                       f"{len(acs)} aircraft · {self._tx_count} msgs")


# ═══════════════════════════════════════════════════════════════════════════════
# Section 4 — Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="ADS-B Interactive Emulator — click the PPI to place aircraft",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--group",  default="239.255.0.1")
    parser.add_argument("--port",   type=int, default=30003)
    parser.add_argument("--iface",  default="127.0.0.1")
    parser.add_argument("--centre", metavar="LAT,LON", default=None,
                        help="Radar centre, e.g. 51.5,-0.5")
    parser.add_argument("--range",  type=float, default=200.0,
                        help="Display range in nm (default: 200)")
    args = parser.parse_args()

    lat, lon = 51.477, -0.461   # default: London Heathrow area
    if args.centre:
        lat, lon = map(float, args.centre.split(","))

    App(args.group, args.port, args.iface, lat, lon, args.range).mainloop()


if __name__ == "__main__":
    main()
