#!/usr/bin/env python3
"""
ADS-B Path Emulator
===================
Click waypoints on the PPI canvas to build a flight route for each aircraft.
Aircraft follow their assigned paths and transmit ADS-B over UDP multicast.

Controls
--------
    Add waypoint    — left-click empty canvas space (adds to the selected aircraft)
    Move waypoint   — drag a waypoint dot
    Delete waypoint — right-click a waypoint dot
    New aircraft    — [+ New Aircraft] in the panel
    Select aircraft — click its row in the panel list

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


# ── Palette ───────────────────────────────────────────────────────────────────

_BG       = "#000000"
_RADAR_BG = "#000a00"
_RING_D   = "#112211"
_RING_B   = "#00aa00"
_SWEEP_C  = "#00ff44"
_TRAIL_C  = "#003300"
_WP_SEL   = "#ffaa00"   # selected aircraft's waypoint dots
_WP_DIM   = "#443300"   # unselected
_PATH_SEL = "#664400"   # selected aircraft's path line
_PATH_DIM = "#221a00"   # unselected
_BLIP_SEL = "#ffff00"
_BLIP     = "#00ff00"
_LBL      = "#aaffaa"
_DIM      = "#336633"
_PANEL_BG = "#0d0d0d"
_BTN_BG   = "#0d1a0d"
_BTN_FG   = "#00cc00"

_CANVAS   = 680
_PANEL    = 240
_HIT_WP   = 10     # waypoint hit-test radius in pixels
_SWEEP    = 36.0   # degrees per second


# ── Coordinate helpers ────────────────────────────────────────────────────────

def _ll_to_xy(lat, lon, cx, cy, r_px, c_lat, c_lon, range_nm):
    scale = r_px / range_nm
    nm_e  = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    nm_n  = (lat - c_lat) * 60.0
    if math.hypot(nm_e, nm_n) > range_nm * 1.02:
        return None
    return cx + nm_e * scale, cy - nm_n * scale


def _xy_to_ll(x, y, cx, cy, r_px, c_lat, c_lon, range_nm):
    scale = r_px / range_nm
    nm_e  = (x - cx) / scale
    nm_n  = (cy - y) / scale
    if math.hypot(nm_e, nm_n) > range_nm:
        return None
    lat = c_lat + nm_n / 60.0
    lon = c_lon + nm_e / (60.0 * math.cos(math.radians(c_lat)))
    return lat, lon


def _distance_nm(lat0, lon0, lat1, lon1):
    dlat = (lat1 - lat0) * 60.0
    dlon = (lon1 - lon0) * 60.0 * math.cos(math.radians((lat0 + lat1) / 2.0))
    return math.hypot(dlat, dlon)


def _bearing(lat0, lon0, lat1, lon1):
    dlat = (lat1 - lat0) * 60.0
    dlon = (lon1 - lon0) * 60.0 * math.cos(math.radians(lat0))
    return math.degrees(math.atan2(dlon, dlat)) % 360.0


# ── Aircraft model ────────────────────────────────────────────────────────────

_counter = [0]


def _next_icao():
    _counter[0] += 1
    return f"FF{_counter[0]:04X}"


def _next_callsign():
    return f"SIM{_counter[0]:03d}"


class WaypointAircraft:
    """
    Aircraft that follows an ordered list of (lat, lon) waypoints.

    Position is linearly interpolated between consecutive waypoints based on
    the configured speed.  When the final waypoint is reached, the aircraft
    either loops back to the first or holds position depending on self.loop.
    """

    __slots__ = ("icao", "callsign", "waypoints", "alt_ft", "speed_kt",
                 "loop", "_seg", "_seg_t", "_lat", "_lon", "_msg_idx")

    def __init__(self, alt_ft=35_000, speed_kt=450):
        self.icao      = _next_icao()
        self.callsign  = _next_callsign()
        self.waypoints = []     # list of (lat, lon)
        self.alt_ft    = alt_ft
        self.speed_kt  = speed_kt
        self.loop      = True
        self._seg      = 0      # index of the segment start waypoint
        self._seg_t    = 0.0    # seconds spent on the current segment
        self._lat      = None
        self._lon      = None
        self._msg_idx  = 0

    @property
    def lat(self):
        if self._lat is not None:
            return self._lat
        return self.waypoints[0][0] if self.waypoints else None

    @property
    def lon(self):
        if self._lon is not None:
            return self._lon
        return self.waypoints[0][1] if self.waypoints else None

    def heading(self):
        """Bearing toward the next waypoint, or 0 if path has fewer than 2 points."""
        if len(self.waypoints) < 2:
            return 0.0
        seg = min(self._seg, len(self.waypoints) - 2)
        return _bearing(*self.waypoints[seg], *self.waypoints[seg + 1])

    def step(self, dt):
        """Advance position along the path by dt seconds."""
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
            lat0, lon0 = wps[seg]
            lat1, lon1 = wps[seg + 1]
            seg_nm   = _distance_nm(lat0, lon0, lat1, lon1)
            seg_time = (seg_nm / self.speed_kt * 3600.0) if self.speed_kt > 0 else 1e9

            if self._seg_t < seg_time or seg_time <= 0:
                t = min(self._seg_t / seg_time, 1.0) if seg_time > 0 else 0.0
                self._lat = lat0 + t * (lat1 - lat0)
                self._lon = lon0 + t * (lon1 - lon0)
                self._seg = seg
                break

            self._seg_t -= seg_time
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

    def next_message(self):
        """Return (label, hex_str) in a 7-message round-robin."""
        seq = self._msg_idx % 7
        self._msg_idx += 1
        lat = self.lat or 0.0
        lon = self.lon or 0.0
        hdg = self.heading()
        if seq == 0:
            return "IDENT", build_identification(self.icao, self.callsign)
        if seq in (1, 4):
            return "POS-E", build_position(self.icao, lat, lon, self.alt_ft, False)
        if seq in (2, 5):
            return "POS-O", build_position(self.icao, lat, lon, self.alt_ft, True)
        return "VEL",   build_velocity(self.icao, self.speed_kt, hdg, 0)


# ── Application ───────────────────────────────────────────────────────────────

class App(tk.Tk):
    """
    Path emulator window.

    Left — PPI canvas.  Click empty space to append a waypoint to the selected
    aircraft; drag a waypoint dot to reposition it; right-click to delete it.
    Right — panel for creating/selecting aircraft, editing their parameters,
    and viewing transmit status.

    The TX background thread writes only to self._tx_status (a plain string);
    the main draw loop reads it and updates the label — no cross-thread
    tkinter calls.
    """

    def __init__(self, group, port, iface, c_lat, c_lon, range_nm):
        super().__init__()
        self.title("ADS-B Path Emulator")
        self.configure(bg=_PANEL_BG)
        self.resizable(False, False)

        self.c_lat    = c_lat
        self.c_lon    = c_lon
        self.range_nm = range_nm
        self.sweep    = 0.0
        self._tick    = time.monotonic()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                              socket.inet_aton(iface))
        self._group = group
        self._port  = port

        self._aircraft: list[WaypointAircraft] = []
        self._selected: WaypointAircraft | None = None
        self._drag_wp = None      # (WaypointAircraft, int) while dragging
        self._lock    = threading.Lock()
        self._tx_count  = 0
        self._tx_status = ""
        self._list_dirty = True   # throttle listbox rebuilds

        self._build_ui()
        threading.Thread(target=self._tx_loop, daemon=True).start()
        self._loop()

    # ── Geometry ──────────────────────────────────────────────────────────────

    def _geom(self):
        cx = cy = _CANVAS // 2
        return cx, cy, cx - 18

    def _to_xy(self, lat, lon):
        cx, cy, r = self._geom()
        return _ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.range_nm)

    def _to_ll(self, x, y):
        cx, cy, r = self._geom()
        return _xy_to_ll(x, y, cx, cy, r, self.c_lat, self.c_lon, self.range_nm)

    def _nearest_wp(self, x, y):
        """Return (aircraft, waypoint_index) for the closest waypoint within _HIT_WP px."""
        best, best_d = None, _HIT_WP
        for ac in self._aircraft:
            for i, (lat, lon) in enumerate(ac.waypoints):
                pt = self._to_xy(lat, lon)
                if pt is None:
                    continue
                d = math.hypot(x - pt[0], y - pt[1])
                if d < best_d:
                    best, best_d = (ac, i), d
        return best

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=_CANVAS, height=_CANVAS,
                            bg=_BG, cursor="crosshair", highlightthickness=0)
        self.cv.pack(side=tk.LEFT, padx=4, pady=4)
        self.cv.bind("<Button-1>",        self._press)
        self.cv.bind("<B1-Motion>",       self._drag)
        self.cv.bind("<ButtonRelease-1>", self._release)
        self.cv.bind("<Button-2>",        self._rclick)
        self.cv.bind("<Button-3>",        self._rclick)
        self.cv.bind("<Motion>",          self._hover)

        pf = tk.Frame(self, bg=_PANEL_BG, width=_PANEL)
        pf.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4), pady=4)
        pf.pack_propagate(False)
        self._build_panel(pf)

    def _sec(self, parent, title):
        f = tk.LabelFrame(parent, text=f" {title} ", bg=_PANEL_BG, fg="#00cc00",
                          font=("Courier", 9, "bold"), relief=tk.GROOVE, bd=1)
        f.pack(fill=tk.X, padx=4, pady=(6, 0))
        return f

    def _btn(self, parent, text, cmd, fg=_BTN_FG):
        return tk.Button(parent, text=text, command=cmd,
                         bg=_BTN_BG, fg=fg, activebackground="#1a331a",
                         activeforeground=fg, font=("Courier", 8, "bold"),
                         relief=tk.FLAT, bd=1, cursor="hand2")

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
        # Aircraft list
        af = self._sec(pf, "Aircraft")
        self._btn(af, "+ New Aircraft", self._new_aircraft).pack(
            fill=tk.X, padx=4, pady=(4, 2))
        self._lb = tk.Listbox(
            af, bg="#050f05", fg="#00ff00", selectbackground="#003300",
            selectforeground="#00ff88", font=("Courier", 8),
            relief=tk.FLAT, bd=0, height=5, activestyle="none")
        self._lb.pack(fill=tk.X, padx=4, pady=(0, 4))
        self._lb.bind("<<ListboxSelect>>", self._lb_select)

        # Edit selected
        ef = self._sec(pf, "Selected")
        self._v_name = tk.StringVar(value="—")
        tk.Label(ef, textvariable=self._v_name, bg=_PANEL_BG, fg="#00cc00",
                 font=("Courier", 8, "bold")).pack(pady=(4, 2))
        self._v_alt  = tk.StringVar(value="35000")
        self._v_spd  = tk.StringVar(value="450")
        self._v_loop = tk.BooleanVar(value=True)
        self._field(ef, "Alt ft",   self._v_alt)
        self._field(ef, "Speed kt", self._v_spd)
        lf = tk.Frame(ef, bg=_PANEL_BG)
        lf.pack(fill=tk.X, padx=4, pady=2)
        tk.Label(lf, text="Loop path", bg=_PANEL_BG, fg="#668866",
                 font=("Courier", 8), width=10, anchor="w").pack(side=tk.LEFT)
        tk.Checkbutton(lf, variable=self._v_loop, bg=_PANEL_BG,
                       fg="#00ff00", selectcolor="#001100",
                       activebackground=_PANEL_BG,
                       command=self._apply_loop).pack(side=tk.LEFT)
        bf = tk.Frame(ef, bg=_PANEL_BG)
        bf.pack(fill=tk.X, padx=4, pady=4)
        self._btn(bf, "Apply", self._update_sel).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))
        self._btn(bf, "Delete", self._delete_aircraft, "#cc4400").pack(
            side=tk.LEFT, fill=tk.X, expand=True)

        # Radar settings
        rf = self._sec(pf, "Radar")
        self._v_clat = tk.StringVar(value=str(self.c_lat))
        self._v_clon = tk.StringVar(value=str(self.c_lon))
        self._v_rnm  = tk.StringVar(value=str(int(self.range_nm)))
        self._field(rf, "Centre lat", self._v_clat)
        self._field(rf, "Centre lon", self._v_clon)
        self._field(rf, "Range nm",   self._v_rnm)
        self._btn(rf, "Apply", self._apply_radar).pack(fill=tk.X, padx=4, pady=4)

        # Transmit status
        sf = self._sec(pf, "Transmit")
        self._v_status = tk.StringVar(value="waiting…")
        tk.Label(sf, textvariable=self._v_status, bg=_PANEL_BG, fg="#668866",
                 font=("Courier", 7), justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=4, pady=4)

        # Hint
        hint = ("L-click canvas → add waypoint\n"
                "Drag dot       → move waypoint\n"
                "R-click dot    → delete waypoint\n"
                "Need 2+ wps to move")
        tk.Label(pf, text=hint, bg=_PANEL_BG, fg="#224422",
                 font=("Courier", 7), justify=tk.LEFT, anchor="w"
                 ).pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=(0, 2))

        # Cursor lat/lon
        self._v_cursor = tk.StringVar()
        tk.Label(pf, textvariable=self._v_cursor, bg=_PANEL_BG, fg="#336633",
                 font=("Courier", 7), justify=tk.LEFT, anchor="w"
                 ).pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=4)

    # ── Mouse events ──────────────────────────────────────────────────────────

    def _press(self, ev):
        hit = self._nearest_wp(ev.x, ev.y)
        if hit:
            ac, idx = hit
            self._select(ac)
            self._drag_wp = (ac, idx)
        else:
            ll = self._to_ll(ev.x, ev.y)
            if ll:
                if self._selected is None:
                    self._new_aircraft()   # auto-create on first canvas click
                with self._lock:
                    self._selected.waypoints.append(ll)
                self._list_dirty = True

    def _drag(self, ev):
        if self._drag_wp is None:
            return
        ll = self._to_ll(ev.x, ev.y)
        if ll:
            ac, idx = self._drag_wp
            with self._lock:
                if idx < len(ac.waypoints):
                    ac.waypoints[idx] = ll

    def _release(self, _ev):
        self._drag_wp = None

    def _hover(self, ev):
        ll = self._to_ll(ev.x, ev.y)
        self._v_cursor.set(f"lat  {ll[0]:+.4f}°\nlon  {ll[1]:+.4f}°" if ll else "")

    def _rclick(self, ev):
        hit = self._nearest_wp(ev.x, ev.y)
        if hit:
            ac, idx = hit
            with self._lock:
                if idx < len(ac.waypoints):
                    ac.waypoints.pop(idx)
            self._list_dirty = True

    # ── Aircraft management ───────────────────────────────────────────────────

    def _new_aircraft(self):
        ac = WaypointAircraft()
        with self._lock:
            self._aircraft.append(ac)
        self._select(ac)
        self._list_dirty = True

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
            self._lb.see(idx)

    def _lb_select(self, _ev):
        sel = self._lb.curselection()
        if sel and sel[0] < len(self._aircraft):
            ac = self._aircraft[sel[0]]
            if ac is not self._selected:
                self._select(ac)

    def _refresh_list(self):
        if not self._list_dirty:
            return
        self._list_dirty = False
        sel_idx = next((i for i, a in enumerate(self._aircraft)
                        if a is self._selected), None)
        self._lb.delete(0, tk.END)
        for ac in self._aircraft:
            wps = len(ac.waypoints)
            self._lb.insert(tk.END, f" {ac.icao}  {ac.callsign}  ({wps} wp)")
        if sel_idx is not None:
            self._lb.selection_set(sel_idx)

    def _update_sel(self):
        ac = self._selected
        if ac is None:
            return
        try:
            with self._lock:
                ac.alt_ft   = int(self._v_alt.get())
                ac.speed_kt = int(self._v_spd.get())
        except ValueError:
            pass

    def _apply_loop(self):
        if self._selected:
            self._selected.loop = self._v_loop.get()

    def _delete_aircraft(self):
        if self._selected is None:
            return
        with self._lock:
            try:
                self._aircraft.remove(self._selected)
            except ValueError:
                pass
        self._selected = None
        self._v_name.set("—")
        self._list_dirty = True

    def _apply_radar(self):
        try:
            self.c_lat    = float(self._v_clat.get())
            self.c_lon    = float(self._v_clon.get())
            self.range_nm = float(self._v_rnm.get())
        except ValueError:
            pass

    # ── Draw loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        now = time.monotonic()
        dt  = now - self._tick
        self._tick = now
        self.sweep = (self.sweep + _SWEEP * dt) % 360.0

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

        # Sweep
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

        # Paths and blips — snapshot under lock
        with self._lock:
            snapshot = [(ac, list(ac.waypoints), ac.lat, ac.lon,
                         ac.heading(), ac._seg)
                        for ac in self._aircraft]

        for ac, wps, lat, lon, hdg, seg in snapshot:
            self._draw_route(cv, ac, wps, lat, lon, hdg, seg)

    def _draw_route(self, cv, ac, wps, lat, lon, hdg, seg):
        sel = ac is self._selected

        # Path lines between waypoints
        pts = [self._to_xy(lat, lon) for lat, lon in wps]
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            if a and b:
                cv.create_line(a[0], a[1], b[0], b[1],
                               fill=(_PATH_SEL if sel else _PATH_DIM),
                               width=(1 if not sel else 1), dash=(4, 4))
        # Close loop line
        if ac.loop and len(pts) >= 2 and pts[0] and pts[-1]:
            cv.create_line(pts[-1][0], pts[-1][1], pts[0][0], pts[0][1],
                           fill=_PATH_DIM, width=1, dash=(2, 6))

        # Waypoint dots and numbers
        for i, pt in enumerate(pts):
            if pt is None:
                continue
            is_target = sel and i == (seg + 1) % max(len(wps), 1)
            color = "#ff6600" if is_target else (_WP_SEL if sel else _WP_DIM)
            cv.create_oval(pt[0]-5, pt[1]-5, pt[0]+5, pt[1]+5,
                           fill=color, outline=color)
            cv.create_text(pt[0]+8, pt[1]-8, text=str(i + 1),
                           fill=(_WP_SEL if sel else _DIM), font=("Courier", 7))

        # Aircraft blip
        if lat is None:
            return
        pt = self._to_xy(lat, lon)
        if pt is None:
            return
        x, y = pt
        fill = _BLIP_SEL if sel else _BLIP
        h    = math.radians(hdg)
        sz   = 9
        verts = []
        for a in (h, h + math.radians(148), h - math.radians(148)):
            verts += [x + sz * math.sin(a), y - sz * math.cos(a)]
        cv.create_polygon(verts, fill=fill, outline="#ffffff" if sel else fill)
        cv.create_text(x+14, y-16, text=ac.callsign,
                       fill=_LBL, font=("Courier", 8, "bold"), anchor="w")
        cv.create_text(x+14, y-6,  text=f"FL{ac.alt_ft//100:03d}",
                       fill=_DIM, font=("Courier", 7), anchor="w")

    # ── TX thread ─────────────────────────────────────────────────────────────

    def _tx_loop(self):
        """
        Background thread: sends one ADS-B message per aircraft per ~80 ms.
        Writes status to self._tx_status; the main thread reads it in _loop().
        """
        while True:
            with self._lock:
                acs = list(self._aircraft)
            if not acs:
                time.sleep(0.1)
                continue
            for ac in acs:
                _, raw = ac.next_message()
                try:
                    self._sock.sendto(f"*{raw};\n".encode(),
                                      (self._group, self._port))
                    self._tx_count += 1
                except OSError:
                    pass
                time.sleep(0.08)
            self._tx_status = (f"{self._group}:{self._port}\n"
                               f"{len(acs)} aircraft · {self._tx_count} msgs")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    _cfg = net_config.load()
    parser = argparse.ArgumentParser(
        description="ADS-B Path Emulator — click waypoints to build flight routes",
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
