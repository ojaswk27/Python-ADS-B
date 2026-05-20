#!/usr/bin/env python3
"""
ADS-B Path Emulator
===================
Click waypoints on the radar canvas to build a flight route.
Aircraft follow their path and transmit ADS-B over UDP multicast.

Controls
--------
    Left-click canvas   add waypoint (auto-creates aircraft if none selected)
    Drag waypoint dot   reposition waypoint
    Right-click dot     delete waypoint
    Panel list          select aircraft
    Apply button        update altitude / speed

Usage
-----
    python path_emulator.py
    python path_emulator.py --centre 51.5,-0.5 --range 150
"""

import argparse
import math
import socket
import threading
import time
import tkinter as tk

import net_config
from aircraft_emulator import build_identification, build_position, build_velocity


# ── Palette (monochrome) ──────────────────────────────────────────────────────

_BG      = "#000000"
_RADAR   = "#050505"
_RING_D  = "#1c1c1c"
_RING_B  = "#444444"
_SWEEP   = "#ffffff"
_SWEEP_T = "#1a1a1a"
_WP      = "#888888"
_WP_SEL  = "#ffffff"
_WP_NEXT = "#aaaaaa"
_PATH    = "#2a2a2a"
_PATH_S  = "#555555"
_BLIP    = "#ffffff"
_BLIP_S  = "#ffffff"
_LBL     = "#aaaaaa"
_DIM     = "#444444"
_PANEL   = "#0c0c0c"
_FG      = "#cccccc"
_FG_DIM  = "#555555"
_SEP     = "#1e1e1e"
_ENTRY   = "#111111"
_BTN     = "#1a1a1a"
_BTN_ACT = "#2a2a2a"

_CANVAS_SZ = 680
_PANEL_W   = 200
_HIT_WP    = 10
_SWEEP_SPD = 36.0   # deg/s


# ── Helpers ───────────────────────────────────────────────────────────────────

def _ll_to_xy(lat, lon, cx, cy, r, c_lat, c_lon, rng):
    s   = r / rng
    nm_e = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    nm_n = (lat - c_lat) * 60.0
    if math.hypot(nm_e, nm_n) > rng * 1.02:
        return None
    return cx + nm_e * s, cy - nm_n * s


def _xy_to_ll(x, y, cx, cy, r, c_lat, c_lon, rng):
    s    = r / rng
    nm_e = (x - cx) / s
    nm_n = (cy - y) / s
    if math.hypot(nm_e, nm_n) > rng:
        return None
    return (c_lat + nm_n / 60.0,
            c_lon + nm_e / (60.0 * math.cos(math.radians(c_lat))))


def _dist_nm(lat0, lon0, lat1, lon1):
    dlat = (lat1 - lat0) * 60.0
    dlon = (lon1 - lon0) * 60.0 * math.cos(math.radians((lat0 + lat1) / 2))
    return math.hypot(dlat, dlon)


def _bearing(lat0, lon0, lat1, lon1):
    dlat = (lat1 - lat0) * 60.0
    dlon = (lon1 - lon0) * 60.0 * math.cos(math.radians(lat0))
    return math.degrees(math.atan2(dlon, dlat)) % 360.0


# ── Aircraft model ────────────────────────────────────────────────────────────

_ctr = [0]

def _new_id():
    _ctr[0] += 1
    return f"FF{_ctr[0]:04X}", f"SIM{_ctr[0]:03d}"


class WaypointAircraft:
    """Follows an ordered waypoint list; position interpolated by speed."""

    __slots__ = ("icao", "callsign", "waypoints", "alt_ft", "speed_kt",
                 "loop", "_seg", "_seg_t", "_lat", "_lon", "_mi")

    def __init__(self, alt_ft=35_000, speed_kt=450):
        self.icao, self.callsign = _new_id()
        self.waypoints = []
        self.alt_ft    = alt_ft
        self.speed_kt  = speed_kt
        self.loop      = True
        self._seg      = 0
        self._seg_t    = 0.0
        self._lat      = None
        self._lon      = None
        self._mi       = 0

    @property
    def lat(self):
        return self._lat if self._lat is not None else (
            self.waypoints[0][0] if self.waypoints else None)

    @property
    def lon(self):
        return self._lon if self._lon is not None else (
            self.waypoints[0][1] if self.waypoints else None)

    def heading(self):
        if len(self.waypoints) < 2:
            return 0.0
        seg = min(self._seg, len(self.waypoints) - 2)
        return _bearing(*self.waypoints[seg], *self.waypoints[seg + 1])

    def step(self, dt):
        wps = self.waypoints
        if len(wps) < 2 or self.speed_kt <= 0:
            if wps:
                self._lat, self._lon = wps[0]
            return
        seg = self._seg
        if seg >= len(wps) - 1:
            if self.loop:
                self._seg = seg = 0
                self._seg_t = 0.0
            else:
                self._lat, self._lon = wps[-1]
                return
        self._seg_t += dt
        while True:
            la, lo = wps[seg]
            lb, lob = wps[seg + 1]
            stime = (_dist_nm(la, lo, lb, lob) / self.speed_kt * 3600.0
                     if self.speed_kt > 0 else 1e9)
            if self._seg_t < stime or stime <= 0:
                t = min(self._seg_t / stime, 1.0) if stime > 0 else 0.0
                self._lat = la + t * (lb - la)
                self._lon = lo + t * (lob - lo)
                self._seg = seg
                break
            self._seg_t -= stime
            seg += 1
            if seg >= len(wps) - 1:
                if self.loop:
                    seg = 0
                    self._seg_t = 0.0
                else:
                    self._lat, self._lon = wps[-1]
                    self._seg = len(wps) - 1
                    self._seg_t = 0.0
                    break

    def next_msg(self):
        seq = self._mi % 7
        self._mi += 1
        lat, lon, hdg = self.lat or 0.0, self.lon or 0.0, self.heading()
        if seq == 0:
            return build_identification(self.icao, self.callsign)
        if seq in (1, 4):
            return build_position(self.icao, lat, lon, self.alt_ft, False)
        if seq in (2, 5):
            return build_position(self.icao, lat, lon, self.alt_ft, True)
        return build_velocity(self.icao, self.speed_kt, hdg, 0)


# ── App ───────────────────────────────────────────────────────────────────────

def _sep(parent):
    tk.Frame(parent, bg=_SEP, height=1).pack(fill=tk.X, padx=0, pady=4)


class App(tk.Tk):

    def __init__(self, group, port, iface, c_lat, c_lon, rng):
        super().__init__()
        self.title("Path Emulator")
        self.configure(bg=_PANEL)
        self.resizable(False, False)

        self.c_lat, self.c_lon, self.rng = c_lat, c_lon, rng
        self.sweep = 0.0
        self._tick = time.monotonic()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                              socket.inet_aton(iface))
        self._group, self._port = group, port

        self._aircraft: list[WaypointAircraft] = []
        self._selected: WaypointAircraft | None = None
        self._drag_wp  = None
        self._lock     = threading.Lock()
        self._tx_count = 0
        self._tx_status = ""
        self._dirty    = True

        self._build_ui()
        threading.Thread(target=self._tx_loop, daemon=True).start()
        self._loop()

    # ── geometry ──────────────────────────────────────────────────────────────

    def _geom(self):
        c = _CANVAS_SZ // 2
        return c, c, c - 18

    def _to_xy(self, lat, lon):
        cx, cy, r = self._geom()
        return _ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    def _to_ll(self, x, y):
        cx, cy, r = self._geom()
        return _xy_to_ll(x, y, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    def _nearest_wp(self, x, y):
        best, bd = None, _HIT_WP
        for ac in self._aircraft:
            for i, (la, lo) in enumerate(ac.waypoints):
                pt = self._to_xy(la, lo)
                if pt and math.hypot(x - pt[0], y - pt[1]) < bd:
                    best, bd = (ac, i), math.hypot(x - pt[0], y - pt[1])
        return best

    # ── UI ────────────────────────────────────────────────────────────────────

    def _lbl(self, parent, text, dim=False):
        tk.Label(parent, text=text, bg=_PANEL,
                 fg=_FG_DIM if dim else _FG,
                 font=("Courier", 8), anchor="w").pack(fill=tk.X, padx=8)

    def _entry_row(self, parent, label, var):
        f = tk.Frame(parent, bg=_PANEL)
        f.pack(fill=tk.X, padx=8, pady=1)
        tk.Label(f, text=label, bg=_PANEL, fg=_FG_DIM,
                 font=("Courier", 8), width=9, anchor="w").pack(side=tk.LEFT)
        tk.Entry(f, textvariable=var, width=8,
                 bg=_ENTRY, fg=_FG, insertbackground=_FG,
                 font=("Courier", 9), relief=tk.FLAT, bd=4
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True)

    def _button(self, parent, text, cmd, padx=8, pady=3):
        b = tk.Button(parent, text=text, command=cmd,
                      bg=_BTN, fg=_FG, activebackground=_BTN_ACT,
                      activeforeground=_FG, font=("Courier", 8),
                      relief=tk.FLAT, bd=0, cursor="hand2", pady=pady)
        b.pack(fill=tk.X, padx=padx, pady=2)
        return b

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=_CANVAS_SZ, height=_CANVAS_SZ,
                            bg=_BG, cursor="crosshair", highlightthickness=0)
        self.cv.pack(side=tk.LEFT)
        self.cv.bind("<Button-1>",        self._press)
        self.cv.bind("<B1-Motion>",       self._drag)
        self.cv.bind("<ButtonRelease-1>", lambda _: setattr(self, "_drag_wp", None))
        self.cv.bind("<Button-2>",        self._rclick)
        self.cv.bind("<Button-3>",        self._rclick)
        self.cv.bind("<Motion>",          self._hover)

        p = tk.Frame(self, bg=_PANEL, width=_PANEL_W)
        p.pack(side=tk.LEFT, fill=tk.Y)
        p.pack_propagate(False)

        # spacer at top
        tk.Frame(p, bg=_PANEL, height=10).pack()

        self._lbl(p, "AIRCRAFT")
        _sep(p)
        self._button(p, "+ New", self._new_ac)

        self._lb = tk.Listbox(p, bg=_ENTRY, fg=_FG, selectbackground="#222222",
                              selectforeground="#ffffff", font=("Courier", 8),
                              relief=tk.FLAT, bd=0, height=5, activestyle="none",
                              highlightthickness=0)
        self._lb.pack(fill=tk.X, padx=8, pady=(4, 0))
        self._lb.bind("<<ListboxSelect>>", self._lb_sel)

        _sep(p)
        self._v_name = tk.StringVar(value="—")
        tk.Label(p, textvariable=self._v_name, bg=_PANEL, fg=_FG,
                 font=("Courier", 8, "bold"), anchor="w").pack(fill=tk.X, padx=8)

        self._v_alt  = tk.StringVar(value="35000")
        self._v_spd  = tk.StringVar(value="450")
        self._v_loop = tk.BooleanVar(value=True)
        self._entry_row(p, "alt ft",   self._v_alt)
        self._entry_row(p, "speed kt", self._v_spd)

        lf = tk.Frame(p, bg=_PANEL)
        lf.pack(fill=tk.X, padx=8, pady=2)
        tk.Label(lf, text="loop", bg=_PANEL, fg=_FG_DIM,
                 font=("Courier", 8), width=9, anchor="w").pack(side=tk.LEFT)
        tk.Checkbutton(lf, variable=self._v_loop, bg=_PANEL,
                       fg=_FG, selectcolor="#111111",
                       activebackground=_PANEL,
                       command=lambda: setattr(self._selected, "loop",
                                               self._v_loop.get())
                       if self._selected else None).pack(side=tk.LEFT)

        bf = tk.Frame(p, bg=_PANEL)
        bf.pack(fill=tk.X, padx=8, pady=4)
        tk.Button(bf, text="Apply", command=self._apply_sel,
                  bg=_BTN, fg=_FG, activebackground=_BTN_ACT,
                  font=("Courier", 8), relief=tk.FLAT, bd=0, cursor="hand2"
                  ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        tk.Button(bf, text="Delete", command=self._del_ac,
                  bg=_BTN, fg="#888888", activebackground=_BTN_ACT,
                  font=("Courier", 8), relief=tk.FLAT, bd=0, cursor="hand2"
                  ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        _sep(p)
        self._v_status = tk.StringVar(value="—")
        tk.Label(p, textvariable=self._v_status, bg=_PANEL, fg=_FG_DIM,
                 font=("Courier", 7), justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=8)

        # hint at bottom
        tk.Frame(p, bg=_PANEL).pack(fill=tk.Y, expand=True)
        _sep(p)
        hint = "click  → waypoint\ndrag   → move\nR-click → delete\n2+ wps to fly"
        tk.Label(p, text=hint, bg=_PANEL, fg="#333333",
                 font=("Courier", 7), justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=8, pady=(0, 8))

        self._v_cur = tk.StringVar()
        tk.Label(p, textvariable=self._v_cur, bg=_PANEL, fg="#444444",
                 font=("Courier", 7), anchor="w"
                 ).pack(fill=tk.X, padx=8, pady=(0, 6))

    # ── mouse ─────────────────────────────────────────────────────────────────

    def _press(self, ev):
        hit = self._nearest_wp(ev.x, ev.y)
        if hit:
            self._select(hit[0])
            self._drag_wp = hit
        else:
            ll = self._to_ll(ev.x, ev.y)
            if ll:
                if self._selected is None:
                    self._new_ac()
                with self._lock:
                    self._selected.waypoints.append(ll)
                self._dirty = True

    def _drag(self, ev):
        if not self._drag_wp:
            return
        ll = self._to_ll(ev.x, ev.y)
        if ll:
            ac, i = self._drag_wp
            with self._lock:
                if i < len(ac.waypoints):
                    ac.waypoints[i] = ll

    def _hover(self, ev):
        ll = self._to_ll(ev.x, ev.y)
        self._v_cur.set(f"{ll[0]:+.4f}  {ll[1]:+.4f}" if ll else "")

    def _rclick(self, ev):
        hit = self._nearest_wp(ev.x, ev.y)
        if hit:
            ac, i = hit
            with self._lock:
                if i < len(ac.waypoints):
                    ac.waypoints.pop(i)
            self._dirty = True

    # ── aircraft mgmt ─────────────────────────────────────────────────────────

    def _new_ac(self):
        ac = WaypointAircraft()
        with self._lock:
            self._aircraft.append(ac)
        self._select(ac)
        self._dirty = True

    def _select(self, ac):
        self._selected = ac
        self._v_name.set(f"{ac.icao}  {ac.callsign}")
        self._v_alt.set(str(ac.alt_ft))
        self._v_spd.set(str(ac.speed_kt))
        self._v_loop.set(ac.loop)
        idx = next((i for i, a in enumerate(self._aircraft) if a is ac), None)
        if idx is not None:
            self._lb.selection_clear(0, tk.END)
            self._lb.selection_set(idx)

    def _lb_sel(self, _ev):
        s = self._lb.curselection()
        if s and s[0] < len(self._aircraft):
            self._select(self._aircraft[s[0]])

    def _refresh_list(self):
        if not self._dirty:
            return
        self._dirty = False
        si = next((i for i, a in enumerate(self._aircraft)
                   if a is self._selected), None)
        self._lb.delete(0, tk.END)
        for ac in self._aircraft:
            self._lb.insert(tk.END,
                f" {ac.icao}  {len(ac.waypoints)} wp")
        if si is not None:
            self._lb.selection_set(si)

    def _apply_sel(self):
        if not self._selected:
            return
        try:
            with self._lock:
                self._selected.alt_ft   = int(self._v_alt.get())
                self._selected.speed_kt = int(self._v_spd.get())
        except ValueError:
            pass

    def _del_ac(self):
        if not self._selected:
            return
        with self._lock:
            try:
                self._aircraft.remove(self._selected)
            except ValueError:
                pass
        self._selected = None
        self._v_name.set("—")
        self._dirty = True

    # ── draw loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        now = time.monotonic()
        dt  = now - self._tick
        self._tick = now
        self.sweep = (self.sweep + _SWEEP_SPD * dt) % 360.0

        with self._lock:
            for ac in self._aircraft:
                ac.step(dt)

        self._refresh_list()
        self._draw()
        if self._tx_status:
            self._v_status.set(self._tx_status)
        self.after(50, self._loop)

    def _draw(self):
        cv = self.cv
        cv.delete("all")
        cx, cy, r = self._geom()

        cv.create_oval(cx-r, cy-r, cx+r, cy+r, fill=_RADAR, outline="")

        # range rings
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

        # sweep
        for off in range(-20, 1):
            ang = math.radians((self.sweep + off) % 360)
            cv.create_line(cx, cy,
                           cx + int(r * math.sin(ang)),
                           cy - int(r * math.cos(ang)),
                           fill=(_SWEEP if off == 0 else _SWEEP_T),
                           width=(1 if off == 0 else 1))

        cv.create_text(6, 6,
                       text=f"{self.c_lat:+.3f}  {self.c_lon:+.3f}  {self.rng:.0f}nm",
                       fill=_DIM, font=("Courier", 7), anchor="nw")

        with self._lock:
            snap = [(ac, list(ac.waypoints), ac.lat, ac.lon,
                     ac.heading(), ac._seg) for ac in self._aircraft]

        for ac, wps, lat, lon, hdg, seg in snap:
            self._draw_route(cv, ac, wps, lat, lon, hdg, seg)

    def _draw_route(self, cv, ac, wps, lat, lon, hdg, seg):
        sel  = ac is self._selected
        pts  = [self._to_xy(la, lo) for la, lo in wps]

        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i+1]
            if a and b:
                cv.create_line(a[0], a[1], b[0], b[1],
                               fill=(_PATH_S if sel else _PATH),
                               dash=(4, 4))
        if ac.loop and len(pts) >= 2 and pts[0] and pts[-1]:
            cv.create_line(pts[-1][0], pts[-1][1], pts[0][0], pts[0][1],
                           fill=_PATH, dash=(2, 6))

        for i, pt in enumerate(pts):
            if not pt:
                continue
            is_tgt = sel and i == (seg + 1) % max(len(wps), 1)
            c = _WP_NEXT if is_tgt else (_WP_SEL if sel else _WP)
            cv.create_oval(pt[0]-4, pt[1]-4, pt[0]+4, pt[1]+4,
                           fill=c, outline="")
            cv.create_text(pt[0]+7, pt[1]-7, text=str(i+1),
                           fill=c, font=("Courier", 7))

        if lat is None:
            return
        pt = self._to_xy(lat, lon)
        if not pt:
            return
        x, y = pt
        h  = math.radians(hdg)
        sz = 8
        v  = []
        for a in (h, h + math.radians(148), h - math.radians(148)):
            v += [x + sz * math.sin(a), y - sz * math.cos(a)]
        cv.create_polygon(v, fill=_BLIP_S if sel else _BLIP, outline="")
        cv.create_text(x+12, y-12, text=ac.callsign,
                       fill=_LBL, font=("Courier", 8, "bold"), anchor="w")
        cv.create_text(x+12, y-2,  text=f"FL{ac.alt_ft//100:03d}",
                       fill=_DIM, font=("Courier", 7), anchor="w")

    # ── TX thread ─────────────────────────────────────────────────────────────

    def _tx_loop(self):
        while True:
            with self._lock:
                acs = list(self._aircraft)
            if not acs:
                time.sleep(0.1)
                continue
            for ac in acs:
                try:
                    raw = ac.next_msg()
                    self._sock.sendto(f"*{raw};\n".encode(),
                                      (self._group, self._port))
                    self._tx_count += 1
                except OSError:
                    pass
                time.sleep(0.08)
            self._tx_status = (f"{self._group}:{self._port}  "
                               f"{len(acs)} ac  {self._tx_count} msgs")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    cfg = net_config.load()
    p   = argparse.ArgumentParser(description="ADS-B Path Emulator",
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
