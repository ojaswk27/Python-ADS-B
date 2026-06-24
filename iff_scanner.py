"""
IFF radar scanner — opens as a Toplevel of the airspace_sim window.

Sweeping antenna at a configurable RPM; at each PRT it picks every in-beam
aircraft (hard-sector beam, |Δaz| ≤ beamwidth/2), filters by capability for the
current interrogation mode, sorts by slant range, caps at MAX_TARGETS, and
appends one decoded summary line per PRT to the reply log.

Reads the sim's aircraft list directly under sim._lock; writes nothing back.
"""

import math
import threading
import time
import tkinter as tk

import iff_protocol as iff
import radar_ui as ui


# ── Defaults / limits ─────────────────────────────────────────────────────────

_DEFAULT_RPM  = 15
_DEFAULT_BW   = 3       # degrees
_DEFAULT_PRT  = 1000    # microseconds
_MIN_PRT_S    = 0.001   # 1 ms — Tk and our loop both stutter below this
_BLIP_DECAY_S  = 4.0    # latched blip fades linearly to invisible over this

_MODE_LABELS = [
    ("Mode 1",            iff.MODE_1),
    ("Mode 2",            iff.MODE_2),
    ("Mode 3/A",          iff.MODE_3A),
    ("Mode C",            iff.MODE_C),
    ("Mode S All-Call",   iff.MODE_S_AC),
    ("Mode S Selective",  iff.MODE_S_SEL),
]


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _bearing_nm(c_lat, c_lon, lat, lon):
    """Return (bearing_deg_from_north, range_nm) of (lat, lon) from (c_lat, c_lon)."""
    nm_n = (lat - c_lat) * 60.0
    nm_e = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    rng  = math.hypot(nm_e, nm_n)
    brg  = math.degrees(math.atan2(nm_e, nm_n)) % 360.0
    return brg, rng


def _angle_diff(a, b):
    """Smallest signed angular difference a−b in [-180, 180]."""
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


# ── Scanner window ────────────────────────────────────────────────────────────

class ScannerWindow(tk.Toplevel):
    """Sweeping PPI + interrogation panel + reply log."""

    def __init__(self, sim):
        super().__init__(sim)
        self.sim = sim                          # the airspace_sim.App
        self.title("IFF Radar Scanner")
        self.configure(bg=ui.PANEL)
        self.resizable(True, True)
        self.minsize(round(560 * ui.SCALE), round(420 * ui.SCALE))

        # Mirror the sim's centre/range so the PPI shares coordinates.
        self.c_lat = sim.c_lat
        self.c_lon = sim.c_lon
        self.rng   = sim.rng

        # Sweep state.  Azimuth is derived from the wall clock + RPM rather
        # than incremented per frame, so the PRT thread (which fires much
        # faster than the Tk 20 Hz redraw) gets the *current* antenna angle
        # rather than the last value the redraw cached.  _sweep_anchor pins
        # the integration: az = (now - anchor_t) * rpm * 6 + anchor_az  (mod 360).
        # Re-pin whenever RPM changes so the sweep doesn't jump.
        self._sweep_anchor_t  = time.monotonic()
        self._sweep_anchor_az = 0.0
        self._sweep_rpm       = _DEFAULT_RPM
        self._tick       = time.monotonic()
        self._cw = self._ch = ui.CANVAS_SZ
        self._fullscreen = False
        self._bg_sig     = None
        self._fg_sig     = None
        self._beam_sig   = None
        self._prt_no     = 0

        # Latched per-aircraft snapshot taken at the moment the beam paints
        # them.  PPI behaviour: blip is FROZEN at the swept position and fades
        # until the next pass refreshes it.  Aircraft never swept = absent.
        # Schema:
        #   icao → (ts, lat, lon, hdg_deg_or_None, color, callsign,
        #            squawk_code, alt_ft, modes_addr, mode, raw_reply_bytes)
        self._latched: dict[str, tuple] = {}
        self._selected_icao: str | None = None     # row clicked in table or blip clicked on PPI
        self._table_dirty = False

        self._lock    = threading.Lock()

        # Tk vars for the interrogation panel
        self._v_mode      = tk.StringVar(value=_MODE_LABELS[2][0])   # default M3/A
        self._v_target    = tk.StringVar(value="(no aircraft)")      # selective addr
        self._v_rpm       = tk.IntVar(value=_DEFAULT_RPM)
        self._v_bw        = tk.DoubleVar(value=_DEFAULT_BW)
        self._v_prt       = tk.IntVar(value=_DEFAULT_PRT)

        self._build_ui()
        self.bind("<F11>",    self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # PRT thread — runs the interrogations in the background; the Tk loop
        # only does rendering and log flushing.
        self._stop = threading.Event()
        threading.Thread(target=self._prt_loop, daemon=True).start()
        self._loop()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=ui.CANVAS_SZ, height=ui.CANVAS_SZ,
                            bg=ui.BG, highlightthickness=0, cursor="crosshair")
        self.cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.cv.bind("<Configure>", self._on_resize)
        self.cv.bind("<Button-1>",  self._on_canvas_click)

        p = ui.make_panel(self)

        # ── INTERROGATION ──
        tk.Frame(p, bg=ui.PANEL, height=round(10 * ui.SCALE)).pack()
        tk.Label(p, text="INTERROGATION", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)

        # Mode dropdown
        labels = [lbl for lbl, _ in _MODE_LABELS]
        mf = tk.Frame(p, bg=ui.PANEL)
        mf.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        tk.Label(mf, text="mode", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        om = tk.OptionMenu(mf, self._v_mode, *labels, command=self._on_mode_change)
        om.config(bg=ui.ENTRY, fg=ui.FG, activebackground=ui.BTN_ACT,
                  font=ui.F_MD, relief=tk.FLAT, bd=0, highlightthickness=0)
        om["menu"].config(bg=ui.ENTRY, fg=ui.FG, font=ui.F_MD)
        om.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Selective target dropdown (only meaningful in MS Selective)
        tf = tk.Frame(p, bg=ui.PANEL)
        tf.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        tk.Label(tf, text="target", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        self._target_om = tk.OptionMenu(tf, self._v_target, "(no aircraft)")
        self._target_om.config(bg=ui.ENTRY, fg=ui.FG, activebackground=ui.BTN_ACT,
                               font=ui.F_MD, relief=tk.FLAT, bd=0, highlightthickness=0)
        self._target_om["menu"].config(bg=ui.ENTRY, fg=ui.FG, font=ui.F_MD)
        self._target_om.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._refresh_target_menu()

        # Sliders
        ui.slider_row(p, "rpm",   self._v_rpm,  5,   30)
        ui.slider_row(p, "bw °",  self._v_bw,   1.0, 10.0, resolution=0.5)
        ui.slider_row(p, "prt µs", self._v_prt, 250, 5000, resolution=50)

        # ── TRACKS ── (one row per latched aircraft, sorted by range)
        ui.sep(p)
        tk.Label(p, text="TRACKS", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)

        # Fixed-width text widget so each row's columns line up.
        table_frame = tk.Frame(p, bg=ui.PANEL)
        table_frame.pack(fill=tk.BOTH, expand=True, padx=ui.PAD, pady=(0, ui.PAD2))

        # Header row in its own widget so it doesn't get clobbered.
        self._tbl_hdr = tk.Label(table_frame, bg=ui.PANEL, fg=ui.FG_DIM,
                                  font=ui.F_SM, anchor="w", justify=tk.LEFT,
                                  text=self._table_row(callsign="CALL",
                                                       sqwk="SQWK",
                                                       rng="RNG",
                                                       brg="BRG",
                                                       alt="ALT",
                                                       age="AGE"))
        self._tbl_hdr.pack(fill=tk.X)

        self._tbl = tk.Text(table_frame, bg=ui.ENTRY, fg=ui.FG, font=ui.F_SM,
                            relief=tk.FLAT, bd=0, wrap=tk.NONE,
                            cursor="hand2", highlightthickness=0,
                            state=tk.DISABLED, height=10)
        self._tbl.pack(fill=tk.BOTH, expand=True)
        self._tbl.bind("<Button-1>", self._on_table_click)
        self._tbl.tag_configure("row", foreground=ui.FG)
        self._tbl.tag_configure("sel", background="#2a2a2a", foreground="#ffffff")

        # ── LAST REPLY ── (raw hex of the selected aircraft's most recent reply)
        ui.sep(p)
        tk.Label(p, text="LAST REPLY", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)
        self._v_detail = tk.StringVar(value="(click an aircraft on the radar or in the table)")
        tk.Label(p, textvariable=self._v_detail, bg=ui.PANEL, fg="#888888",
                 font=ui.F_SM, anchor="w", justify=tk.LEFT, wraplength=ui.PANEL_W - 2 * ui.PAD
                 ).pack(fill=tk.X, padx=ui.PAD, pady=(0, ui.PAD))

    # ── geometry ──────────────────────────────────────────────────────────────

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

    def _on_close(self):
        self._stop.set()
        self.destroy()

    # ── sweep azimuth ─────────────────────────────────────────────────────────

    def _azimuth_now(self) -> float:
        """Antenna azimuth at this exact instant.

        Derived from the wall clock + RPM so the PRT thread (which fires far
        faster than the 20 Hz Tk redraw) sees the *current* angle and not the
        last value the redraw cached — otherwise targets that lived in the
        ~4.5°/frame gap between consecutive Tk-frame azimuths would be missed
        on every revolution.
        """
        rpm = self._v_rpm.get()
        if rpm != self._sweep_rpm:
            # Re-pin so the sweep is continuous across an RPM change.
            self._sweep_anchor_az = self._azimuth_at(time.monotonic(),
                                                    self._sweep_rpm)
            self._sweep_anchor_t  = time.monotonic()
            self._sweep_rpm       = rpm
        return self._azimuth_at(time.monotonic(), rpm)

    def _azimuth_at(self, t: float, rpm: float) -> float:
        return (self._sweep_anchor_az +
                (t - self._sweep_anchor_t) * rpm * 6.0) % 360.0

    # ── interrogation panel callbacks ─────────────────────────────────────────

    def _current_mode(self) -> int:
        label = self._v_mode.get()
        for lbl, code in _MODE_LABELS:
            if lbl == label:
                return code
        return iff.MODE_3A

    def _on_mode_change(self, _label=None):
        # Selective needs an up-to-date target menu; cheap to refresh on every change.
        self._refresh_target_menu()

    def _refresh_target_menu(self):
        """Populate the selective-target dropdown from the sim's aircraft list."""
        menu = self._target_om["menu"]
        menu.delete(0, "end")
        with self.sim._lock:
            entries = [(ac.icao, ac.callsign, ac.modes_addr)
                       for ac in self.sim._aircraft]
        if not entries:
            menu.add_command(label="(no aircraft)",
                             command=lambda: self._v_target.set("(no aircraft)"))
            self._v_target.set("(no aircraft)")
            return
        cur = self._v_target.get()
        seen = False
        for icao, call, addr in entries:
            label = f"{icao}  {call}  ({addr:06X})"
            menu.add_command(label=label,
                             command=lambda l=label: self._v_target.set(l))
            if label == cur:
                seen = True
        if not seen:
            self._v_target.set(entries[0][0] + f"  {entries[0][1]}  "
                               f"({entries[0][2]:06X})")

    def _selected_target_addr(self):
        """Parse the 6-hex address out of the selective-target dropdown label."""
        s = self._v_target.get()
        i = s.rfind("(")
        j = s.rfind(")")
        if i < 0 or j < 0:
            return None
        try:
            return int(s[i+1:j], 16)
        except ValueError:
            return None

    # ── PRT loop (background thread) ──────────────────────────────────────────

    def _prt_loop(self):
        """Fire interrogations at the configured PRT.  Touches no Tk APIs."""
        while not self._stop.is_set():
            prt_s = max(self._v_prt.get() * 1e-6, _MIN_PRT_S)
            time.sleep(prt_s)

            mode    = self._current_mode()
            bw      = self._v_bw.get()
            half_bw = bw / 2.0

            # Snapshot ground truth (everything we need to latch a blip)
            # under the sim lock; the per-aircraft tuple becomes the source of
            # truth for both the reply and the latched display snapshot.
            sel_addr = self._selected_target_addr() if mode == iff.MODE_S_SEL else None
            with self.sim._lock:
                c_lat, c_lon, rng_max = self.sim.c_lat, self.sim.c_lon, self.sim.rng
                snap = []
                for ac in self.sim._aircraft:
                    if ac.lat is None or ac.lon is None:
                        continue
                    if not ac.has_xpdr(mode):
                        continue
                    if mode == iff.MODE_S_SEL and (sel_addr is None or
                                                   ac.modes_addr != sel_addr):
                        continue
                    hdg = ac.heading() if len(ac.waypoints) >= 2 else None
                    snap.append((ac.icao, ac.callsign, ac.color, ac.lat, ac.lon,
                                 hdg, ac.alt_ft, ac.modes_addr,
                                 ac.mode1, ac.mode2, ac.mode3a))
            az = self._azimuth_now()

            # Hard-sector filter + sort by range
            hits = []
            for icao, call, col, lat, lon, hdg, alt_ft, msa, m1, m2, m3a in snap:
                brg, rng = _bearing_nm(c_lat, c_lon, lat, lon)
                if rng > rng_max:
                    continue
                if abs(_angle_diff(brg, az)) > half_bw:
                    continue
                hits.append((rng, icao, call, col, lat, lon, hdg, alt_ft,
                             msa, m1, m2, m3a))

            hits.sort(key=lambda t: t[0])
            hits = hits[:iff.MAX_TARGETS]

            # Build the per-target records, keeping the per-hit code so we can
            # latch it onto the track table row.
            records = []
            codes = []
            for rng, icao, call, col, lat, lon, hdg, alt_ft, msa, m1, m2, m3a in hits:
                if mode == iff.MODE_1:
                    code = m1
                elif mode == iff.MODE_2:
                    code = m2
                elif mode == iff.MODE_3A:
                    code = m3a
                elif mode == iff.MODE_C:
                    code = iff.encode_mode_c(alt_ft)
                else:
                    code = 0
                codes.append(code)
                records.append(iff.TargetRecord(range_nm=rng, code=code,
                                                modes_addr=msa))

            self._prt_no = (self._prt_no + 1) & 0xFFFF
            raw = iff.build_reply(prt_no=self._prt_no, azimuth_deg=az,
                                  mode=mode, targets=records)

            # Latch a frozen snapshot of each hit at the azimuth instant.  The
            # PPI draws from these snapshots so blips stay where they were
            # painted and fade in place; the track-table panel reads from the
            # same dict so the two views are always consistent.
            now = time.monotonic()
            with self._lock:
                for (rng, icao, call, col, lat, lon, hdg, alt_ft,
                     msa, _m1, _m2, _m3a), code in zip(hits, codes):
                    self._latched[icao] = (now, lat, lon, hdg, col, call,
                                           code, alt_ft, msa, mode, raw)
                if hits:
                    self._table_dirty = True

    # ── render loop (Tk main thread) ─────────────────────────────────────────

    def _loop(self):
        if self._stop.is_set():
            return
        now = time.monotonic()
        dt  = now - self._tick
        self._tick = now

        # (Azimuth is derived on demand from the wall clock — no per-frame
        # integration is needed; the PRT thread and _draw both call
        # _azimuth_now() to get the current value.)

        self._draw()
        # Throttle the table rebuild to ~5 Hz: it walks _latched + sorts +
        # re-renders Text rows, so doing it every Tk frame would burn CPU
        # for no visible gain (age column ticks at 0.1 s granularity).
        if (self._table_dirty or
                (now - getattr(self, "_table_last", 0.0)) >= 0.2):
            self._flush_table()
            self._table_last = now
        self.after(50, self._loop)

    # ── draw ──────────────────────────────────────────────────────────────────

    def _view_sig(self):
        return (round(self.c_lat, 6), round(self.c_lon, 6),
                round(self.rng, 3), self._cw, self._ch)

    def _draw(self):
        cv = self.cv
        cx, cy, r = ui.geom(self._cw, self._ch)
        sf = ui.scale_for(self._cw, self._ch)

        # Background: rings + grid — cached
        sig = self._view_sig()
        if sig != self._bg_sig:
            cv.delete("bg")
            ui.draw_radar_frame(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                sf, tag="bg")
            ui.draw_latlon_grid(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                tag="bg")
            self._bg_sig = sig
            self._beam_sig = None
            self._fg_sig = None

        # Beam wedge — one pie slice centred on antenna azimuth
        bw = self._v_bw.get()
        az = self._azimuth_now()
        beam_sig = (round(az, 1), round(bw, 2), self._cw, self._ch)
        if beam_sig != self._beam_sig:
            cv.delete("beam")
            # Tk arc angles are CCW from +x.  Antenna az is CW from +y (north).
            # Convert: tk_start = 90 - az - bw/2  (degrees)
            tk_start = (90.0 - az - bw / 2.0) % 360.0
            cv.create_arc(cx - r, cy - r, cx + r, cy + r,
                          start=tk_start, extent=bw,
                          fill="#1a1a1a", outline="", style="pieslice",
                          tags="beam")
            self._beam_sig = beam_sig

        # Foreground: latched blips only.  Each is frozen at the (lat, lon, hdg)
        # the beam painted it at and fades linearly over _BLIP_DECAY_S.  An
        # aircraft that has never been swept is invisible until the beam first
        # crosses it.  Re-detection moves the latched snapshot to a new spot,
        # which is exactly the classical PPI look.
        now = time.monotonic()
        with self._lock:
            latched = list(self._latched.items())

        # Drop fully-decayed entries so the dict doesn't grow without bound
        live = [(icao, snap) for icao, snap in latched
                if now - snap[0] <= _BLIP_DECAY_S]
        if len(live) != len(latched):
            with self._lock:
                live_keys = {icao for icao, _ in live}
                self._latched = {k: v for k, v in self._latched.items()
                                 if k in live_keys}

        # Quantised fade level so frames skip re-render between fade steps
        fg_sig = tuple(
            (icao, round(snap[1], 5), round(snap[2], 5),
             None if snap[3] is None else round(snap[3], 1),
             int((now - snap[0]) * 8))     # 8 fade steps per second
            for icao, snap in live
        )
        if fg_sig == self._fg_sig:
            return
        self._fg_sig = fg_sig

        cv.delete("fg")
        for icao, (ts, lat, lon, hdg, col, callsign) in live:
            pt = self._to_xy(lat, lon)
            if pt is None:
                continue
            x, y = pt
            age = now - ts
            faded = ui.shade(col, max(0.0, 1.0 - age / _BLIP_DECAY_S))
            if hdg is None:
                rad = ui.BLIP_SZ * sf
                cv.create_oval(x - rad, y - rad, x + rad, y + rad,
                               fill=faded, outline="", tags="fg")
            else:
                ui.draw_blip(cv, x, y, math.radians(hdg), faded, sf, tag="fg")
            cv.create_text(x + ui.LBL_DX * sf, y - ui.LBL_DY * sf,
                           text=callsign, fill=faded,
                           font=ui.sfont(ui.PT_MD, sf, bold=True),
                           anchor="w", tags="fg")

    # ── track table + last-reply pane ────────────────────────────────────────

    # Fixed column widths — keep total ≤ panel width so the row fits without wrap.
    _COL_FMT = "{callsign:<8} {sqwk:>5} {rng:>6} {brg:>5} {alt:>6} {age:>4}"

    def _table_row(self, *, callsign, sqwk, rng, brg, alt, age):
        return self._COL_FMT.format(callsign=callsign[:8], sqwk=sqwk,
                                    rng=rng, brg=brg, alt=alt, age=age)

    def _flush_table(self):
        """Rebuild the track table from _latched (newest snapshot per ICAO)."""
        # Snapshot the latched dict + the centre under the appropriate locks.
        now = time.monotonic()
        with self._lock:
            entries = []
            for icao, snap in self._latched.items():
                (ts, lat, lon, _hdg, _col, callsign, code, alt_ft,
                 msa, mode, _raw) = snap
                age = now - ts
                if age > _BLIP_DECAY_S:
                    continue
                brg, rng = _bearing_nm(self.c_lat, self.c_lon, lat, lon)
                entries.append((rng, icao, callsign, code, alt_ft, msa, mode,
                                brg, age))
            self._table_dirty = False
        # Drop the live ordering on the user (closest first).
        entries.sort(key=lambda t: t[0])

        # Build the rows
        self._tbl.config(state=tk.NORMAL)
        self._tbl.delete("1.0", tk.END)
        self._row_icaos: list[str] = []
        for rng, icao, callsign, code, alt_ft, msa, mode, brg, age in entries:
            # Squawk is octal for classic modes, hex 6-digit ICAO for Mode S
            if mode in (iff.MODE_S_AC, iff.MODE_S_SEL):
                sqwk = f"{msa:06X}"
            else:
                sqwk = f"{code & 0xFFFF:04o}"
            alt_s = f"FL{alt_ft//100:03d}" if alt_ft is not None else "—"
            row = self._table_row(callsign=callsign or icao,
                                  sqwk=sqwk,
                                  rng=f"{rng:5.1f}",
                                  brg=f"{brg:4.0f}°",
                                  alt=alt_s,
                                  age=f"{age:3.1f}s")
            tag = "sel" if icao == self._selected_icao else "row"
            self._tbl.insert(tk.END, row + "\n", tag)
            self._row_icaos.append(icao)
        if not entries:
            self._tbl.insert(tk.END, "  (no tracks)\n", "row")
        self._tbl.config(state=tk.DISABLED)

        # Update the last-reply detail pane for the currently-selected aircraft.
        self._refresh_detail()

    def _refresh_detail(self):
        """Show the most recent raw reply hex for the currently selected ICAO."""
        icao = self._selected_icao
        if icao is None:
            self._v_detail.set("(click an aircraft on the radar or in the table)")
            return
        with self._lock:
            snap = self._latched.get(icao)
        if snap is None:
            self._v_detail.set(f"{icao}: no recent reply (decayed)")
            return
        ts, _lat, _lon, _hdg, _col, callsign, _code, _alt, _msa, mode, raw = snap
        head = f"{callsign}  {iff.MODE_NAMES.get(mode, '?')}"
        self._v_detail.set(head + "\n" + iff.format_hex(raw))

    def _on_table_click(self, ev):
        """Click selects that aircraft (used to drive the detail pane)."""
        idx = self._tbl.index(f"@{ev.x},{ev.y}")
        line_no = int(idx.split(".")[0])
        if 1 <= line_no <= len(self._row_icaos):
            self._selected_icao = self._row_icaos[line_no - 1]
            self._table_dirty = True

    def _on_canvas_click(self, ev):
        """Hit-test against the latched blip positions; select the nearest one."""
        with self._lock:
            entries = [(icao, snap[1], snap[2]) for icao, snap in self._latched.items()]
        best = None
        best_d = ui.HIT_WP * ui.scale_for(self._cw, self._ch) * 3.0
        for icao, lat, lon in entries:
            pt = self._to_xy(lat, lon)
            if pt is None:
                continue
            d = math.hypot(ev.x - pt[0], ev.y - pt[1])
            if d < best_d:
                best_d = d
                best = icao
        self._selected_icao = best          # may be None to deselect
        self._table_dirty = True
