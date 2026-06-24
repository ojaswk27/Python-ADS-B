#!/usr/bin/env python3
"""
Airspace Simulator
==================
Click / drag waypoints on the radar canvas to build flight routes; aircraft
follow them.  This is the *simulator* — it owns ground truth but does NOT
broadcast anything on the network.  An interrogating IFF radar scanner opens
in a second window and reads positions directly from this process under a
shared lock.

Forked from path_emulator.py minus all UDP transmit threads, plus per-aircraft
IFF state (Mode 1/2/3-A squawks, Mode S address, transponder capability flags).

Controls
--------
    Left-click canvas   add a single waypoint (auto-creates aircraft if none)
    Click-drag canvas   draw a freehand path (samples points as you drag)
    Drag waypoint dot   reposition that waypoint
    Right-click dot     delete waypoint  /  R-click empty: toggle crosshair
    loop checkbox       close the path into a loop (off = open path)
    Panel list          select aircraft
    address / callsign  edit ICAO + callsign (Enter or click away to apply)
    IFF fields          Mode 1, Mode 2, Mode 3/A squawks (octal); Mode S addr
    capability flags    M1 / M2 / M3-A / MC / MS transponder on/off
    alt / speed sliders update altitude / speed live
    F11 / Esc           toggle / leave fullscreen (radar autoscales)

Usage
-----
    python airspace_sim.py
    python airspace_sim.py --centre 51.5,-0.5 --range 150 --declination 1.5
"""

import argparse
import math
import random
import threading
import time
import tkinter as tk

import iff_scanner
import radar_ui as ui


# ── Path-specific palette ─────────────────────────────────────────────────────

_WP      = "#888888"
_WP_SEL  = "#ffffff"
_WP_NEXT = "#aaaaaa"
_PATH    = "#2a2a2a"
_PATH_S  = "#555555"
_BLIP    = "#ffffff"
_BLIP_S  = "#ffffff"
_LBL     = "#aaaaaa"

# Freehand drawing: place waypoints at this fixed pixel spacing along the
# drawn stroke.  Points are interpolated along each motion segment so spacing
# stays uniform regardless of mouse polling rate or drag speed.
_DRAW_SPACING_PX = round(9 * ui.SCALE)

# Cursor crosshair arm length (px, before scale_for).  Short so each mouse
# move only invalidates a small region of canvas, not two disc-spanning lines.
_CURSOR_ARM_PX = 22 * ui.SCALE


# ── Helpers ───────────────────────────────────────────────────────────────────

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
    return f"FF{_ctr[0]:04X}", f"SIM{_ctr[0]:03d}", _ctr[0]


class WaypointAircraft:
    """Follows an ordered waypoint list; position interpolated by speed.

    Carries per-aircraft IFF state for the radar scanner:
      mode1/mode2/mode3a — 12-bit octal squawks
      modes_addr        — 24-bit Mode S address (defaults to int(icao, 16))
      xpdr1/2/3a/c/s    — capability flags; if False, this aircraft does not
                          reply to that interrogation mode (non-cooperative).
    """

    __slots__ = ("icao", "callsign", "track_no", "color",
                 "waypoints", "alt_ft", "speed_kt",
                 "loop", "_seg", "_seg_t", "_lat", "_lon", "_mi",
                 "mode1", "mode2", "mode3a", "modes_addr",
                 "xpdr1", "xpdr2", "xpdr3a", "xpdrC", "xpdrS")

    # Map IFF protocol mode codes → which capability flag gates them.
    _MODE_FLAG = {1: "xpdr1", 2: "xpdr2", 3: "xpdr3a", 4: "xpdrC",
                  5: "xpdrS", 6: "xpdrS"}

    def __init__(self, alt_ft=35_000, speed_kt=450):
        self.icao, self.callsign, self.track_no = _new_id()
        self.color     = ui.random_color()
        self.waypoints = []
        self.alt_ft    = alt_ft
        self.speed_kt  = speed_kt
        self.loop      = False   # open path by default; tick "loop" to close it
        self._seg      = 0
        self._seg_t    = 0.0
        self._lat      = None
        self._lon      = None
        self._mi       = 0

        # IFF: random defaults; user can override via the per-aircraft panel.
        self.mode1      = random.randint(0, 0o7777)
        self.mode2      = random.randint(0, 0o7777)
        self.mode3a     = random.randint(0, 0o7777)
        self.modes_addr = int(self.icao, 16) & 0xFFFFFF
        self.xpdr1 = self.xpdr2 = self.xpdr3a = self.xpdrC = self.xpdrS = True

    def has_xpdr(self, mode_code: int) -> bool:
        """Does this aircraft reply to the given IFF protocol mode?"""
        return getattr(self, self._MODE_FLAG.get(mode_code, ""), False)

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

# ── App ───────────────────────────────────────────────────────────────────────

class App(tk.Tk):

    def __init__(self, c_lat, c_lon, rng, declination=0.0):
        super().__init__()
        self.title("Airspace Simulator")
        self.configure(bg=ui.PANEL)
        self.resizable(True, True)
        self.minsize(round(420 * ui.SCALE), round(360 * ui.SCALE))

        self.c_lat, self.c_lon, self.rng = c_lat, c_lon, rng
        self.declination = declination
        self._tick = time.monotonic()
        self._cw = self._ch = ui.CANVAS_SZ      # live canvas size (autoscale)
        self._fullscreen = False
        self._cursor = None                     # (x, y, lat, lon) under pointer
        self._cursor_on = True                  # crosshair visibility (right-click toggles)

        self._aircraft: list[WaypointAircraft] = []
        self._selected: WaypointAircraft | None = None
        self._drag_wp   = None
        self._draw_from = None      # last sampled pixel while freehand-drawing
        self._lock      = threading.Lock()
        self._dirty     = True
        self._bg_sig    = None    # view signature the cached background was drawn for
        self._routes_dirty = True # rebuild route polylines on next frame
        self._fg_sig    = None    # snapshot of dynamic state the fg layer was drawn for

        self._build_ui()
        self.bind("<F11>",    self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)
        # Open the IFF scanner as a Toplevel sharing this sim's aircraft list.
        self.scanner = iff_scanner.ScannerWindow(self)
        self._loop()

    # ── coordinate helpers ────────────────────────────────────────────────────

    def _to_xy(self, lat, lon):
        cx, cy, r = ui.geom(self._cw, self._ch)
        return ui.ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    def _to_ll(self, x, y):
        cx, cy, r = ui.geom(self._cw, self._ch)
        return ui.xy_to_ll(x, y, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    def _mag(self, true_deg):
        """Convert a true heading/track to magnetic via the configured declination."""
        return (true_deg - self.declination) % 360.0

    def _nearest_wp(self, x, y, only=None):
        """Nearest waypoint within the hit radius. If `only` is given, search
        just that aircraft's waypoints; otherwise search all aircraft."""
        best, bd = None, ui.HIT_WP * ui.scale_for(self._cw, self._ch)
        acs = [only] if only is not None else self._aircraft
        for ac in acs:
            for i, (la, lo) in enumerate(ac.waypoints):
                pt = self._to_xy(la, lo)
                if pt and math.hypot(x - pt[0], y - pt[1]) < bd:
                    best, bd = (ac, i), math.hypot(x - pt[0], y - pt[1])
        return best

    # ── UI ────────────────────────────────────────────────────────────────────

    def _button(self, parent, text, cmd):
        b = ui.flat_button(parent, text, cmd)
        b.pack(fill=tk.X, padx=ui.PAD, pady=round(2 * ui.SCALE))
        return b

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=ui.CANVAS_SZ, height=ui.CANVAS_SZ,
                            bg=ui.BG, cursor="crosshair", highlightthickness=0)
        self.cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.cv.bind("<Button-1>",        self._press)
        self.cv.bind("<B1-Motion>",       self._drag)
        self.cv.bind("<ButtonRelease-1>", self._release)
        self.cv.bind("<Button-2>",        self._rclick)
        self.cv.bind("<Button-3>",        self._rclick)
        self.cv.bind("<Motion>",          self._hover)
        self.cv.bind("<Leave>",           lambda _: setattr(self, "_cursor", None))
        self.cv.bind("<Configure>",       self._on_resize)

        p = ui.make_panel(self)

        tk.Frame(p, bg=ui.PANEL, height=round(10 * ui.SCALE)).pack()
        tk.Label(p, text="AIRCRAFT", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)
        self._button(p, "+ New", self._new_ac)

        self._lb = tk.Listbox(p, bg=ui.ENTRY, fg=ui.FG,
                              selectbackground="#222222",
                              selectforeground="#ffffff", font=ui.F_MD,
                              relief=tk.FLAT, bd=0, height=5,
                              activestyle="none", highlightthickness=0)
        self._lb.pack(fill=tk.X, padx=ui.PAD, pady=(ui.PAD2, 0))
        self._lb.bind("<<ListboxSelect>>", self._lb_sel)

        ui.sep(p)
        self._v_name = tk.StringVar(value="—")
        tk.Label(p, textvariable=self._v_name, bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_BLD, anchor="w").pack(fill=tk.X, padx=ui.PAD)

        self._v_icao = tk.StringVar()
        self._v_call = tk.StringVar()
        e_icao = ui.entry_row(p, "address",  self._v_icao)
        e_call = ui.entry_row(p, "callsign", self._v_call)
        for e in (e_icao, e_call):
            e.bind("<Return>",   self._apply_id)
            e.bind("<FocusOut>", self._apply_id)

        self._v_alt  = tk.IntVar(value=35000)
        self._v_spd  = tk.IntVar(value=450)
        self._v_loop = tk.BooleanVar(value=False)
        ui.slider_row(p, "alt ft",   self._v_alt, -1000, 50000, 25,
                      command=self._apply_sel)
        ui.slider_row(p, "speed kt", self._v_spd,     0,  4088,  1,
                      command=self._apply_sel)

        lf = tk.Frame(p, bg=ui.PANEL)
        lf.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        tk.Label(lf, text="loop", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        tk.Checkbutton(lf, variable=self._v_loop, bg=ui.PANEL,
                       fg=ui.FG, selectcolor=ui.ENTRY,
                       activebackground=ui.PANEL,
                       command=self._toggle_loop).pack(side=tk.LEFT)

        # ── IFF state ─────────────────────────────────────────────────────────
        ui.sep(p)
        tk.Label(p, text="IFF", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)

        # Squawk fields (octal, 4 digits) + Mode S address (24-bit hex, 6 digits)
        self._v_m1   = tk.StringVar()
        self._v_m2   = tk.StringVar()
        self._v_m3a  = tk.StringVar()
        self._v_msa  = tk.StringVar()
        e_m1  = ui.entry_row(p, "M1 sqwk",  self._v_m1)
        e_m2  = ui.entry_row(p, "M2 sqwk",  self._v_m2)
        e_m3a = ui.entry_row(p, "M3A sqwk", self._v_m3a)
        e_msa = ui.entry_row(p, "MS addr",  self._v_msa)
        for e in (e_m1, e_m2, e_m3a, e_msa):
            e.bind("<Return>",   self._apply_iff)
            e.bind("<FocusOut>", self._apply_iff)

        # Capability flags — five checkboxes in a 2-row grid
        self._v_x1  = tk.BooleanVar(value=True)
        self._v_x2  = tk.BooleanVar(value=True)
        self._v_x3a = tk.BooleanVar(value=True)
        self._v_xc  = tk.BooleanVar(value=True)
        self._v_xs  = tk.BooleanVar(value=True)
        cf = tk.Frame(p, bg=ui.PANEL)
        cf.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        def _cap(label, var, col, row):
            tk.Checkbutton(cf, text=label, variable=var,
                           bg=ui.PANEL, fg=ui.FG, selectcolor=ui.ENTRY,
                           activebackground=ui.PANEL, font=ui.F_SM,
                           command=self._apply_caps
                           ).grid(row=row, column=col, sticky="w", padx=2)
        _cap("M1",  self._v_x1,  0, 0)
        _cap("M2",  self._v_x2,  1, 0)
        _cap("M3A", self._v_x3a, 2, 0)
        _cap("MC",  self._v_xc,  0, 1)
        _cap("MS",  self._v_xs,  1, 1)

        ui.flat_button(p, "Delete track", self._del_ac, fg="#888888"
                       ).pack(fill=tk.X, padx=ui.PAD, pady=(ui.PAD, ui.PAD2))

        ui.flat_button(p, "Reset positions", self._reset_positions,
                       bg=ui.BTN_RED, fg="#ffffff", active=ui.BTN_RED_A
                       ).pack(fill=tk.X, padx=ui.PAD, pady=(0, ui.PAD))

        tk.Frame(p, bg=ui.PANEL).pack(fill=tk.Y, expand=True)
        ui.sep(p)
        hint = ("click     → add point\n"
                "drag      → draw path\n"
                "drag dot  → move point\n"
                "R-click   → delete point /\n"
                "            toggle crosshair\n"
                "loop ☐    → close path")
        tk.Label(p, text=hint, bg=ui.PANEL, fg="#333333",
                 font=ui.F_SM, justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=ui.PAD, pady=(0, ui.PAD))

        self._v_cur = tk.StringVar()
        tk.Label(p, textvariable=self._v_cur, bg=ui.PANEL, fg="#444444",
                 font=ui.F_SM, anchor="w"
                 ).pack(fill=tk.X, padx=ui.PAD, pady=(0, round(6 * ui.SCALE)))

    # ── mouse ─────────────────────────────────────────────────────────────────

    def _press(self, ev):
        # 1) A waypoint of the already-selected aircraft → reposition it.
        if self._selected is not None:
            hit = self._nearest_wp(ev.x, ev.y, only=self._selected)
            if hit:
                self._drag_wp = hit
                return
        # 2) A waypoint of another aircraft → only select it; you must select
        #    an aircraft before you can shift its waypoints.
        hit = self._nearest_wp(ev.x, ev.y)
        if hit:
            self._select(hit[0])
            return
        # 3) Empty space → drop the first point and arm freehand drawing;
        #    subsequent drag motion appends more points to the path.
        ll = self._to_ll(ev.x, ev.y)
        if ll:
            if self._selected is None:
                self._new_ac()
            with self._lock:
                self._selected.waypoints.append(ll)
            self._draw_from = (ev.x, ev.y)
            self._dirty = self._routes_dirty = True

    def _drag(self, ev):
        if self._drag_wp:
            # Repositioning a single existing waypoint.
            ll = self._to_ll(ev.x, ev.y)
            if ll:
                ac, i = self._drag_wp
                with self._lock:
                    if i < len(ac.waypoints):
                        ac.waypoints[i] = ll
                self._routes_dirty = True
            return
        if self._draw_from is not None and self._selected is not None:
            # Freehand drawing: step evenly along the segment from the last
            # placed point to the cursor, dropping a waypoint every
            # _DRAW_SPACING_PX.  Interpolating (rather than sampling one point
            # per motion event) keeps spacing uniform no matter how far the
            # cursor jumped between events.  The sub-spacing remainder is
            # carried over via _draw_from so spacing stays even across events.
            lx, ly = self._draw_from
            dist = math.hypot(ev.x - lx, ev.y - ly)
            if dist >= _DRAW_SPACING_PX:
                ux, uy = (ev.x - lx) / dist, (ev.y - ly) / dist
                pts = []
                px, py = lx, ly
                for _ in range(int(dist // _DRAW_SPACING_PX)):
                    px += ux * _DRAW_SPACING_PX
                    py += uy * _DRAW_SPACING_PX
                    ll = self._to_ll(px, py)
                    if ll:
                        pts.append(ll)
                if pts:
                    with self._lock:
                        self._selected.waypoints.extend(pts)
                    self._dirty = self._routes_dirty = True
                self._draw_from = (px, py)

    def _release(self, _ev=None):
        self._drag_wp   = None
        self._draw_from = None

    def _hover(self, ev):
        ll = self._to_ll(ev.x, ev.y)
        if ll:
            self._cursor = (ev.x, ev.y, ll[0], ll[1])
            self._v_cur.set(f"{ll[0]:+.4f}  {ll[1]:+.4f}")
        else:
            self._cursor = None
            self._v_cur.set("")

    def _on_resize(self, ev):
        self._cw, self._ch = ev.width, ev.height

    def _rclick(self, ev):
        # Right-click on a waypoint deletes it; on empty space it toggles the
        # cursor crosshair on/off.
        hit = self._nearest_wp(ev.x, ev.y)
        if hit:
            ac, i = hit
            with self._lock:
                if i < len(ac.waypoints):
                    ac.waypoints.pop(i)
            self._dirty = self._routes_dirty = True
        else:
            self._cursor_on = not self._cursor_on

    # ── aircraft mgmt ─────────────────────────────────────────────────────────

    def _new_ac(self):
        ac = WaypointAircraft()
        with self._lock:
            self._aircraft.append(ac)
        self._select(ac)
        self._dirty = True

    def _select(self, ac):
        self._selected = ac
        self._routes_dirty = True   # selection changes route highlight colour
        self._v_name.set(f"{ac.icao}  {ac.callsign}")
        self._v_icao.set(ac.icao)
        self._v_call.set(ac.callsign)
        self._v_alt.set(ac.alt_ft)
        self._v_spd.set(int(ac.speed_kt))
        self._v_loop.set(ac.loop)
        # IFF fields
        self._v_m1.set(f"{ac.mode1:04o}")
        self._v_m2.set(f"{ac.mode2:04o}")
        self._v_m3a.set(f"{ac.mode3a:04o}")
        self._v_msa.set(f"{ac.modes_addr:06X}")
        self._v_x1.set(ac.xpdr1)
        self._v_x2.set(ac.xpdr2)
        self._v_x3a.set(ac.xpdr3a)
        self._v_xc.set(ac.xpdrC)
        self._v_xs.set(ac.xpdrS)
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
            self._lb.insert(tk.END, f" {ac.icao}  {len(ac.waypoints)} wp")
        if si is not None:
            self._lb.selection_set(si)

    def _toggle_loop(self):
        if self._selected:
            self._selected.loop = self._v_loop.get()
            self._routes_dirty = True   # closing leg appears/disappears

    def _apply_sel(self, _val=None):
        if not self._selected:
            return
        with self._lock:
            self._selected.alt_ft   = self._v_alt.get()
            self._selected.speed_kt = self._v_spd.get()

    def _apply_id(self, _ev=None):
        """Apply edited ICAO address / callsign to the selected aircraft."""
        ac = self._selected
        if not ac:
            return
        icao = self._v_icao.get().strip().upper()
        call = self._v_call.get().strip().upper()
        try:
            valid_icao = len(icao) == 6 and int(icao, 16) >= 0
        except ValueError:
            valid_icao = False
        with self._lock:
            if valid_icao:
                ac.icao = icao
            if call:
                ac.callsign = call[:8]
        # Reflect normalised / rejected values back into the fields.
        self._v_icao.set(ac.icao)
        self._v_call.set(ac.callsign)
        self._v_name.set(f"{ac.icao}  {ac.callsign}")
        self._dirty = True

    def _apply_iff(self, _ev=None):
        """Apply edited IFF codes (octal squawks + Mode S hex addr)."""
        ac = self._selected
        if not ac:
            return
        def parse_oct(s, fallback):
            try:
                v = int(s.strip(), 8)
                return v if 0 <= v <= 0o7777 else fallback
            except ValueError:
                return fallback
        def parse_hex(s, fallback):
            try:
                v = int(s.strip(), 16)
                return v if 0 <= v <= 0xFFFFFF else fallback
            except ValueError:
                return fallback
        with self._lock:
            ac.mode1      = parse_oct(self._v_m1.get(),  ac.mode1)
            ac.mode2      = parse_oct(self._v_m2.get(),  ac.mode2)
            ac.mode3a     = parse_oct(self._v_m3a.get(), ac.mode3a)
            ac.modes_addr = parse_hex(self._v_msa.get(), ac.modes_addr)
        # Reflect normalised values back into the fields.
        self._v_m1.set(f"{ac.mode1:04o}")
        self._v_m2.set(f"{ac.mode2:04o}")
        self._v_m3a.set(f"{ac.mode3a:04o}")
        self._v_msa.set(f"{ac.modes_addr:06X}")

    def _apply_caps(self):
        """Apply edited transponder capability flags."""
        ac = self._selected
        if not ac:
            return
        with self._lock:
            ac.xpdr1  = self._v_x1.get()
            ac.xpdr2  = self._v_x2.get()
            ac.xpdr3a = self._v_x3a.get()
            ac.xpdrC  = self._v_xc.get()
            ac.xpdrS  = self._v_xs.get()

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
        self._dirty = self._routes_dirty = True

    def _reset_positions(self):
        """Rewind every aircraft to the start of its path."""
        with self._lock:
            for ac in self._aircraft:
                ac._seg   = 0
                ac._seg_t = 0.0
                ac._lat   = None   # lat/lon properties fall back to waypoints[0]
                ac._lon   = None

    # ── draw loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        now = time.monotonic()
        dt  = now - self._tick
        self._tick = now

        with self._lock:
            for ac in self._aircraft:
                ac.step(dt)

        self._refresh_list()
        self._draw()
        self.after(50, self._loop)

    def _view_sig(self):
        """Signature of everything the static background depends on."""
        return (round(self.c_lat, 6), round(self.c_lon, 6),
                round(self.rng, 3), self._cw, self._ch)

    def _draw(self):
        cv = self.cv
        cx, cy, r = ui.geom(self._cw, self._ch)
        sf = ui.scale_for(self._cw, self._ch)

        # ── Layer 1: static background (rings + grid) — only on view change ──
        sig = self._view_sig()
        if sig != self._bg_sig:
            cv.delete("bg")
            ui.draw_radar_frame(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                sf, tag="bg")
            ui.draw_latlon_grid(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                tag="bg")
            self._bg_sig = sig
            self._routes_dirty = True          # reproject routes for new view
            self._fg_sig = None                # bg rebuild → force fg recreation

        with self._lock:
            snap = [(ac, list(ac.waypoints), ac.lat, ac.lon, ac.heading())
                    for ac in self._aircraft]

        # ── Layer 2: route polylines — only when waypoints/view change ───────
        if self._routes_dirty:
            cv.delete("route")
            for ac, wps, _lat, _lon, _hdg in snap:
                self._draw_route(cv, ac, wps, cx, cy, r, sf)
            self._routes_dirty = False

        # ── Layer 3: blips, labels, cursor — only when something moved ───────
        # Recreated last each frame so they naturally stack above bg/route;
        # route is rebuilt whenever bg is, so the bg<route<fg order always holds.
        # Skip the whole delete+recreate when the dynamic state is byte-identical
        # to the previous frame (idle aircraft, no cursor movement).
        fg_sig = self._fg_signature(snap, sf)
        if fg_sig != self._fg_sig:
            cv.delete("fg")
            for ac, wps, lat, lon, hdg in snap:
                self._draw_target(cv, ac, lat, lon, hdg, sf)
            self._draw_cursor(cv, cx, cy, r, sf)
            self._fg_sig = fg_sig

    def _fg_signature(self, snap, sf):
        """Cheap hash of everything the fg layer depends on."""
        # round lat/lon to ~1m (5 dp) to avoid trivial-position-flip redraws
        ac_sig = tuple(
            (id(ac), ac.alt_ft, round(ac.speed_kt, 1),
             None if lat is None else round(lat, 5),
             None if lon is None else round(lon, 5),
             round(hdg, 1))
            for ac, _wps, lat, lon, hdg in snap
        )
        cur = (self._cursor_on,
               None if self._cursor is None else
               (self._cursor[0], self._cursor[1]))   # px ints already
        return (ac_sig, cur, round(sf, 2),
                id(self._selected))                  # selection drives label colour

    def _draw_cursor(self, cv, cx, cy, r, sf):
        """Short toggleable crosshair around the cursor.

        The arms span only ±_CURSOR_ARM_PX (scaled) instead of the full disc, so
        every mouse-move only dirties a small rectangle around the pointer
        rather than two disc-spanning strips of canvas.
        """
        if not (self._cursor_on and self._cursor):
            return
        x, y, lat, lon = self._cursor
        arm = round(_CURSOR_ARM_PX * sf)
        cv.create_line(x, y - arm, x, y + arm, fill=ui.SEP, tags="fg")
        cv.create_line(x - arm, y, x + arm, y, fill=ui.SEP, tags="fg")
        o = round(6 * ui.SCALE * sf)
        cv.create_text(x + o, y - o,
                       text=f"{lat:+.4f}  {lon:+.4f}",
                       fill=ui.FG, font=ui.sfont(ui.PT_SM, sf), anchor="sw", tags="fg")

    def _draw_route(self, cv, ac, wps, cx, cy, r, sf):
        """Draw one aircraft's path as a continuous line (cached 'route' layer).

        Projects inline with ll_to_xy (geom hoisted by the caller) so a dense
        freehand stroke reads as a smooth curve.  Solid, in the target's colour,
        dimmed when not selected; breaks where a point leaves the disc.
        """
        leg = ac.color if ac is self._selected else _PATH_S
        pts = [ui.ll_to_xy(la, lo, cx, cy, r, self.c_lat, self.c_lon, self.rng)
               for la, lo in wps]
        run = []
        for pt in pts:
            if pt is None:
                if len(run) >= 4:
                    cv.create_line(run, fill=leg, tags="route")
                run = []
            else:
                run += [pt[0], pt[1]]
        if len(run) >= 4:
            cv.create_line(run, fill=leg, tags="route")
        if ac.loop and len(pts) >= 2 and pts[0] and pts[-1]:
            cv.create_line(pts[-1][0], pts[-1][1], pts[0][0], pts[0][1],
                           fill=leg, tags="route")

    def _draw_target(self, cv, ac, lat, lon, hdg, sf):
        """Draw one aircraft's blip and labels (per-frame 'fg' layer)."""
        if lat is None:
            return
        pt = self._to_xy(lat, lon)
        if not pt:
            return
        x, y = pt
        col = ac.color
        dx, dy = ui.LBL_DX * sf, ui.LBL_DY * sf
        ui.draw_blip(cv, x, y, math.radians(hdg), col, sf, tag="fg")
        cv.create_text(x + dx, y - dy, text=ac.callsign, fill=col,
                       font=ui.sfont(ui.PT_MD, sf, bold=True), anchor="w", tags="fg")
        cv.create_text(x + dx, y - round(2 * ui.SCALE * sf),
                       text=f"FL{ac.alt_ft//100:03d}  {self._mag(hdg):03.0f}°M",
                       fill=ui.DIM, font=ui.sfont(ui.PT_SM, sf), anchor="w", tags="fg")

    # ── fullscreen ────────────────────────────────────────────────────────────

    def _toggle_fullscreen(self, _ev=None):
        self._fullscreen = not self._fullscreen
        self.attributes("-fullscreen", self._fullscreen)

    def _exit_fullscreen(self, _ev=None):
        self._fullscreen = False
        self.attributes("-fullscreen", False)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Airspace simulator + IFF radar",
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__)
    p.add_argument("--centre", metavar="LAT,LON", default=None)
    p.add_argument("--range",  type=float, default=200.0)
    p.add_argument("--declination", type=float, default=0.0,
                   help="Magnetic declination °E for true→magnetic heading (default: 0)")
    args = p.parse_args()
    lat, lon = 51.477, -0.461
    if args.centre:
        lat, lon = map(float, args.centre.split(","))
    App(lat, lon, args.range, args.declination).mainloop()


if __name__ == "__main__":
    main()
