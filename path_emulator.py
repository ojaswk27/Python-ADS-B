#!/usr/bin/env python3
"""
ADS-B Path Emulator
===================
Click waypoints on the radar canvas to build a flight route.
Aircraft follow their path and transmit ADS-B over UDP multicast.

Controls
--------
    Left-click canvas   add a single waypoint (auto-creates aircraft if none)
    Click-drag canvas   draw a freehand path (samples points as you drag)
    Drag waypoint dot   reposition that waypoint
    Right-click dot     delete waypoint
    loop checkbox       close the path into a loop (off = open path)
    Panel list          select aircraft
    address / callsign  edit ICAO + callsign (Enter or click away to apply)
    alt / speed sliders update altitude / speed live
    F11 / Esc           toggle / leave fullscreen (radar autoscales)
    Hover               crosshair shows exact lat/lon under the pointer

Outputs
-------
    ADS-B raw hex   → UDP multicast (dump1090 framing)
    ASTERIX CAT021  → unicast --asterix-host:--asterix-port (1 block/s)
    Radar position  → 8 × 12-byte messages (msg-id 0x1306), 500 ms apart at start

Usage
-----
    python path_emulator.py
    python path_emulator.py --centre 51.5,-0.5 --range 150
    python path_emulator.py --declination 1.5 --asterix-host 192.168.1.20 --asterix-port 8600
"""

import argparse
import math
import socket
import struct
import threading
import time
import tkinter as tk

import cat21
import net_config
import radar_ui as ui
from aircraft_emulator import build_identification, build_position, build_velocity


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
    """Follows an ordered waypoint list; position interpolated by speed."""

    __slots__ = ("icao", "callsign", "track_no", "color",
                 "waypoints", "alt_ft", "speed_kt",
                 "loop", "_seg", "_seg_t", "_lat", "_lon", "_mi")

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

class App(tk.Tk):

    def __init__(self, group, port, iface, c_lat, c_lon, rng,
                 declination=0.0, asx_host="127.0.0.1", asx_port=8600):
        super().__init__()
        self.title("Path Emulator")
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

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                              socket.inet_aton(iface))
        self._group, self._port = group, port

        # Unicast socket for ASTERIX CAT021 + the radar-position message.
        self._asx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                                       socket.IPPROTO_UDP)
        self._asx_dst  = (asx_host, asx_port)

        self._aircraft: list[WaypointAircraft] = []
        self._selected: WaypointAircraft | None = None
        self._drag_wp   = None
        self._draw_from = None      # last sampled pixel while freehand-drawing
        self._lock      = threading.Lock()
        self._tx_count  = 0
        self._asx_count = 0
        self._tx_status = ""
        self._dirty     = True
        self._bg_sig    = None    # view signature the cached background was drawn for
        self._routes_dirty = True # rebuild route polylines on next frame
        self._fg_sig    = None    # snapshot of dynamic state the fg layer was drawn for

        self._build_ui()
        self.bind("<F11>",    self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)
        threading.Thread(target=self._tx_loop,         daemon=True).start()
        threading.Thread(target=self._asterix_loop,    daemon=True).start()
        threading.Thread(target=self._radar_pos_burst, daemon=True).start()
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

        ui.flat_button(p, "Delete track", self._del_ac, fg="#888888"
                       ).pack(fill=tk.X, padx=ui.PAD, pady=(ui.PAD, ui.PAD2))

        ui.flat_button(p, "Reset positions", self._reset_positions,
                       bg=ui.BTN_RED, fg="#ffffff", active=ui.BTN_RED_A
                       ).pack(fill=tk.X, padx=ui.PAD, pady=(0, ui.PAD))

        ui.sep(p)
        self._v_status = tk.StringVar(value="—")
        tk.Label(p, textvariable=self._v_status, bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_SM, justify=tk.LEFT, anchor="w"
                 ).pack(fill=tk.X, padx=ui.PAD)

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
        if self._tx_status:
            self._v_status.set(self._tx_status)
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
            self._tx_status = (
                f"ADS-B {self._group}:{self._port}  {self._tx_count} msg\n"
                f"CAT21 {self._asx_dst[0]}:{self._asx_dst[1]}  {self._asx_count} blk")

    # ── ASTERIX CAT021 output ─────────────────────────────────────────────────

    def _asterix_loop(self):
        """Emit a CAT021 data block (one record per active target) ~1/s."""
        while True:
            time.sleep(1.0)
            with self._lock:
                snap = [(ac.track_no, ac.icao, ac.callsign, ac.lat, ac.lon,
                         ac.alt_ft, ac.speed_kt, ac.heading())
                        for ac in self._aircraft if ac.lat is not None]
            records = [
                cat21.target_record(
                    sac=0, sic=1, track=track_no, icao=icao, callsign=call,
                    lat=lat, lon=lon, alt_ft=alt, speed_kt=spd,
                    track_deg=hdg, mag_hdg_deg=self._mag(hdg))
                for track_no, icao, call, lat, lon, alt, spd, hdg in snap
            ]
            if not records:
                continue
            try:
                self._asx_sock.sendto(cat21.build_message(records), self._asx_dst)
                self._asx_count += 1
            except OSError:
                pass

    def _radar_pos_burst(self):
        """Announce the radar's own position: 8 × 12-byte messages, 500 ms apart.

        Struct (little-endian): uint16 size=12, uint16 msg_id=0x1306,
        float32 latitude, float32 longitude.
        """
        pkt = struct.pack("<HHff", 12, 0x1306, self.c_lat, self.c_lon)
        for _ in range(8):                       # 8 × 500 ms = 4 s
            try:
                self._asx_sock.sendto(pkt, self._asx_dst)
            except OSError:
                pass
            time.sleep(0.5)

    # ── fullscreen ────────────────────────────────────────────────────────────

    def _toggle_fullscreen(self, _ev=None):
        self._fullscreen = not self._fullscreen
        self.attributes("-fullscreen", self._fullscreen)

    def _exit_fullscreen(self, _ev=None):
        self._fullscreen = False
        self.attributes("-fullscreen", False)


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
    p.add_argument("--declination", type=float, default=0.0,
                   help="Magnetic declination °E for true→magnetic heading (default: 0)")
    p.add_argument("--asterix-host", default=cfg["asterix_host"],
                   help=f"Unicast IP for CAT021 + radar-position output "
                        f"(default: {cfg['asterix_host']})")
    p.add_argument("--asterix-port", type=int, default=cfg["asterix_port"],
                   help=f"Unicast port for CAT021 + radar-position output "
                        f"(default: {cfg['asterix_port']})")
    args = p.parse_args()
    lat, lon = 51.477, -0.461
    if args.centre:
        lat, lon = map(float, args.centre.split(","))
    App(args.group, args.port, args.iface, lat, lon, args.range,
        args.declination, args.asterix_host, args.asterix_port).mainloop()


if __name__ == "__main__":
    main()
