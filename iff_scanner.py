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
_BLIP_DECAY_S  = 4.0    # sim latched blip fades linearly to grey over this
_ADSB_FADE_S   = 5.0    # ADS-B blip fades toward grey once messages stop

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


# ── Scanner ───────────────────────────────────────────────────────────────────

class Scanner:
    """Sweeping radar + interrogation panel + track table, mounted onto the
    airspace_sim window's existing canvas and side panel.

    Not a Tk widget — owns no window of its own.  The sim calls draw_overlay()
    from its render loop and try_select_blip() from its press handler.  Scanner
    selection is shared with the sim's _selected attribute, so clicking a blip
    or a track row also populates the AIRCRAFT panel.
    """

    def __init__(self, sim, canvas, panel):
        self.sim   = sim                         # the airspace_sim.App
        self.cv    = canvas                      # the shared PPI canvas
        self._panel = panel                      # the shared side panel

        # Sweep state.  Azimuth is derived from the wall clock + RPM rather
        # than incremented per frame, so the PRT thread (which fires much
        # faster than the Tk 20 Hz redraw) gets the *current* antenna angle
        # rather than the last value the redraw cached.  _sweep_anchor pins
        # the integration: az = (now - anchor_t) * rpm * 6 + anchor_az  (mod 360).
        # Re-pin whenever RPM changes so the sweep doesn't jump.
        self._sweep_anchor_t  = time.monotonic()
        self._sweep_anchor_az = 0.0
        self._sweep_rpm       = _DEFAULT_RPM
        self._beam_sig   = None
        self._blip_sig   = None
        self._prt_no     = 0

        # Latched per-aircraft snapshot taken at the moment the beam paints
        # them.  PPI behaviour: blip is FROZEN at the swept position and fades
        # until the next pass refreshes it.  Aircraft never swept = absent.
        # Schema:
        #   icao → (ts, lat, lon, hdg_deg_or_None, color, callsign,
        #            squawk_code, alt_ft, modes_addr, mode, raw_reply_bytes)
        self._latched: dict[str, tuple] = {}
        # ADS-B tracks are not sweep-latched — they render live from their
        # decoder position with an ADS-B-message-age fade.  This holds the
        # extra reply-metadata per ICAO so the track table + LAST REPLY
        # pane stay in sync:  icao → (col, callsign, code, alt_ft, msa, mode, raw)
        self._adsb_meta: dict[str, tuple] = {}
        self._table_dirty = False
        self._table_last  = 0.0
        self._row_icaos: list[str] = []

        self._lock = threading.Lock()

        # Tk vars for the interrogation panel
        self._v_mode         = tk.StringVar(value=_MODE_LABELS[2][0])   # default M3/A
        self._v_target       = tk.StringVar(value="(no aircraft)")      # selective addr
        self._v_use_external = tk.BooleanVar(value=False)               # external mode toggle
        self._v_rpm          = tk.IntVar(value=_DEFAULT_RPM)
        self._v_bw           = tk.DoubleVar(value=_DEFAULT_BW)
        self._v_prt          = tk.IntVar(value=_DEFAULT_PRT)

        # External mode state: populated by the interrogation UDP receiver.
        # Persists indefinitely once received; the toggle chooses whether we
        # actually use it.  Selective addr is optional; None → no override.
        self._external_mode:       int | None = None
        self._external_target:     int | None = None    # 24-bit ICAO
        self._ext_status_snap = "(no messages yet)"

        # Plain-Python mirrors of the Tk slider vars so the PRT thread never
        # calls tk.Variable.get() (which requires the Tk mainloop on 3.13+).
        # The Tk render loop (main thread) refreshes these each frame.
        self._rpm_snap        = _DEFAULT_RPM
        self._bw_snap         = float(_DEFAULT_BW)
        self._prt_s_snap      = _DEFAULT_PRT * 1e-6
        self._use_external_snap = False
        self._mode_label_snap = _MODE_LABELS[2][0]

        # Reply-sender socket + destination.  Opened lazily from the sim's
        # config when the first reply is ready to go out.
        self._reply_sock = None
        self._reply_dst  = None
        self._reply_out_count = 0
        self._reply_out_err   = 0

        self._build_panel()

        # PRT thread — runs the interrogations in the background.
        self._stop = threading.Event()
        threading.Thread(target=self._prt_loop, daemon=True).start()

    # The scanner's "selected aircraft" mirrors the sim's: clicking a blip,
    # a track row, or an aircraft in the listbox all reach the same state.
    @property
    def _selected_icao(self):
        ac = self.sim._selected
        return ac.icao if ac is not None else None

    def _select_by_icao(self, icao):
        ac = next((a for a in self.sim._aircraft if a.icao == icao), None)
        if ac is not None:
            self.sim._select(ac)
            self._table_dirty = True

    # Centre / range tracked from the sim (in case the sim ever lets you move them).
    @property
    def c_lat(self): return self.sim.c_lat
    @property
    def c_lon(self): return self.sim.c_lon
    @property
    def rng(self):   return self.sim.rng

    def stop(self):
        """Called by the sim when its window closes."""
        self._stop.set()
        # Clean up the UDP endpoints if we opened them.
        recv = getattr(self, "_interrogation_rx", None)
        if recv is not None:
            recv.stop()
        if self._reply_sock is not None:
            try:
                self._reply_sock.close()
            except OSError:
                pass

    def configure_udp(self, cfg: dict):
        """Wire the scanner into UDP: start the interrogation receiver and
        open the reply-send socket.  cfg is a net_config dict."""
        import udp_endpoints as udp
        import iff_interrogation as ii
        # Reply-send socket (multicast or unicast per cfg).
        try:
            self._reply_sock, self._reply_dst = udp.open_send(
                cfg["iff_reply_host"], cfg["iff_reply_port"],
                cfg["iff_reply_transport"], cfg["iff_reply_iface"])
        except OSError as e:
            print(f"[scanner] reply send socket failed: {e}")

        # Interrogation-input receiver.
        self._interrogation_rx = ii.Receiver(
            cfg["iff_interrogation_host"],
            cfg["iff_interrogation_port"],
            cfg["iff_interrogation_transport"],
            cfg["iff_interrogation_iface"],
            on_message=self.on_external_interrogation,
        )
        self._interrogation_rx.start()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_panel(self):
        p = self._panel

        # ── INTERROGATION ── top spacer matches the left panel's opening frame
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

        # External-mode toggle: when on, mode comes from incoming interrogation
        # messages (received via UDP); the dropdown becomes advisory.
        ef = tk.Frame(p, bg=ui.PANEL)
        ef.pack(fill=tk.X, padx=ui.PAD, pady=(0, ui.PAD2))
        tk.Label(ef, text="external", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        tk.Checkbutton(ef, variable=self._v_use_external, bg=ui.PANEL,
                       fg=ui.FG, selectcolor=ui.ENTRY,
                       activebackground=ui.PANEL).pack(side=tk.LEFT)
        self._v_ext_status = tk.StringVar(value="(no messages yet)")
        tk.Label(ef, textvariable=self._v_ext_status, bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_SM, anchor="w").pack(side=tk.LEFT, padx=(ui.PAD2, 0))

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
        cx, cy, r = ui.geom(self.sim._cw, self.sim._ch)
        return ui.ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    # ── sweep azimuth ─────────────────────────────────────────────────────────

    def _azimuth_now(self) -> float:
        """Antenna azimuth at this exact instant.

        Derived from the wall clock + RPM so the PRT thread (which fires far
        faster than the 20 Hz Tk redraw) sees the *current* angle and not the
        last value the redraw cached — otherwise targets that lived in the
        ~4.5°/frame gap between consecutive Tk-frame azimuths would be missed
        on every revolution.
        """
        rpm = self._rpm_snap
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
        # External toggle wins if enabled AND we've received at least one msg.
        # Uses the plain-Python snapshots so this is safe from the PRT thread.
        if self._use_external_snap and self._external_mode is not None:
            return self._external_mode
        for lbl, code in _MODE_LABELS:
            if lbl == self._mode_label_snap:
                return code
        return iff.MODE_3A

    def _external_selective_addr(self):
        """Selective target when external mode is driving.  None if unset."""
        if self._use_external_snap:
            return self._external_target
        return None

    def _on_mode_change(self, _label=None):
        # Selective needs an up-to-date target menu; cheap to refresh on every change.
        self._refresh_target_menu()

    def on_external_interrogation(self, msg):
        """Called by the interrogation Receiver on every parsed packet.

        Runs on the receiver thread — writes ONLY plain Python attributes.
        The status label is a Tk StringVar and can't be set safely from a
        non-Tk thread; tick() on the main thread copies _ext_status_snap
        into the var.
        """
        self._external_mode = msg.mode
        if msg.mode in (iff.MODE_S_AC, iff.MODE_S_SEL):
            self._external_target = msg.modes_addr
        else:
            self._external_target = None
        mode_name = iff.MODE_NAMES.get(msg.mode, f"?{msg.mode}")
        addr_s = f" {msg.modes_addr:06X}" if msg.modes_addr is not None else ""
        self._ext_status_snap = f"{mode_name}{addr_s}"

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
        """Fire interrogations at the configured PRT.  Touches no Tk APIs —
        all slider values come from the plain-Python snapshots refreshed by
        tick() on the Tk main thread."""
        while not self._stop.is_set():
            prt_s = max(self._prt_s_snap, _MIN_PRT_S)
            time.sleep(prt_s)

            mode    = self._current_mode()
            bw      = self._bw_snap
            half_bw = bw / 2.0

            # Snapshot ground truth (everything we need to latch a blip)
            # under the sim lock; the per-aircraft tuple becomes the source of
            # truth for both the reply and the latched display snapshot.
            # External toggle can override the selective address too.
            if mode == iff.MODE_S_SEL:
                sel_addr = (self._external_selective_addr() or
                            self._selected_target_addr())
            else:
                sel_addr = None
            # ADS-B tracks live at their live position and don't latch on
            # sweep — mark them so the latch loop can skip them.
            import adsb_source as _asrc
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
                    is_adsb = isinstance(ac, _asrc.AdsbAircraft)
                    snap.append((ac.icao, ac.callsign, ac.color, ac.lat, ac.lon,
                                 hdg, ac.alt_ft, ac.modes_addr,
                                 ac.mode1, ac.mode2, ac.mode3a, is_adsb))
            az = self._azimuth_now()

            # Hard-sector filter + sort by range
            hits = []
            for icao, call, col, lat, lon, hdg, alt_ft, msa, m1, m2, m3a, is_adsb in snap:
                brg, rng = _bearing_nm(c_lat, c_lon, lat, lon)
                if rng > rng_max:
                    continue
                if abs(_angle_diff(brg, az)) > half_bw:
                    continue
                hits.append((rng, icao, call, col, lat, lon, hdg, alt_ft,
                             msa, m1, m2, m3a, is_adsb))

            hits.sort(key=lambda t: t[0])
            hits = hits[:iff.MAX_TARGETS]

            # Build the per-target records, keeping the per-hit code so we can
            # latch it onto the track table row.
            records = []
            codes = []
            for rng, icao, call, col, lat, lon, hdg, alt_ft, msa, m1, m2, m3a, _is_adsb in hits:
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

            # Send the reply block on the wire whenever the beam painted at
            # least one target.  (Empty replies would be a firehose at PRT
            # rates; we skip them.)
            if hits and self._reply_sock is not None:
                try:
                    self._reply_sock.sendto(raw, self._reply_dst)
                    self._reply_out_count += 1
                except OSError:
                    self._reply_out_err += 1

            # Latch a frozen snapshot of each hit at the azimuth instant.  The
            # PPI draws from these snapshots so blips stay where they were
            # painted and fade in place; the track-table panel reads from the
            # same dict so the two views are always consistent.
            now = time.monotonic()
            latched_any = False
            with self._lock:
                for (rng, icao, call, col, lat, lon, hdg, alt_ft,
                     msa, _m1, _m2, _m3a, is_adsb), code in zip(hits, codes):
                    # ADS-B tracks don't sweep-latch; they render live from
                    # their decoder position with an ADS-B-message-age fade
                    # (handled by draw_overlay).  Track-table row and reply
                    # metadata still land on _adsb_meta so we can show them.
                    if is_adsb:
                        self._adsb_meta[icao] = (col, call, code, alt_ft, msa, mode, raw)
                    else:
                        self._latched[icao] = (now, lat, lon, hdg, col, call,
                                               code, alt_ft, msa, mode, raw)
                        latched_any = True
                if hits:
                    self._table_dirty = True

    # ── render loop (Tk main thread) ─────────────────────────────────────────

    def tick(self):
        """Called by the sim's _loop each frame.  Refreshes the plain-Python
        snapshots of Tk vars (safe to read here since we're on the main
        thread), then handles throttled table flushes."""
        if self._stop.is_set():
            return

        # Refresh snapshots for the PRT thread.  These reads are cheap.
        try:
            self._rpm_snap          = self._v_rpm.get()
            self._bw_snap           = float(self._v_bw.get())
            self._prt_s_snap        = self._v_prt.get() * 1e-6
            self._use_external_snap = self._v_use_external.get()
            self._mode_label_snap   = self._v_mode.get()
            # Push the receiver-thread status text into its Tk var.
            if self._v_ext_status.get() != self._ext_status_snap:
                self._v_ext_status.set(self._ext_status_snap)
        except Exception:
            pass

        now = time.monotonic()
        # Throttle the table rebuild to ~5 Hz: it walks _latched + sorts +
        # re-renders Text rows; doing it every Tk frame burns CPU for no
        # visible gain (age column ticks at 0.1 s granularity).
        if self._table_dirty or (now - self._table_last) >= 0.2:
            self._flush_table()
            self._table_last = now

    # ── draw overlay (called by the sim's _draw) ──────────────────────────────

    def draw_overlay(self, cx, cy, r, sf):
        """Paint the scanner-specific layers onto the shared canvas.

        Two tagged layers added on top of the sim's bg + route:
          - "beam"  : the sweep wedge (one pie-slice; redrawn when az moves)
          - "blip"  : latched scanner targets (fade linearly over _BLIP_DECAY_S)

        The sim handles bg/route/fg lifetimes.  We own beam and blip.
        """
        cv = self.cv

        # Beam wedge — one pie slice centred on antenna azimuth
        bw = self._bw_snap
        az = self._azimuth_now()
        beam_sig = (round(az, 1), round(bw, 2), self.sim._cw, self.sim._ch)
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

        # Latched blips: each frozen at the (lat, lon, hdg) the beam painted
        # it at, fading linearly over _BLIP_DECAY_S.  An aircraft never swept
        # is invisible; re-detection moves the snapshot — classical PPI look.
        now = time.monotonic()
        with self._lock:
            latched = list(self._latched.items())

        live = [(icao, snap) for icao, snap in latched
                if now - snap[0] <= _BLIP_DECAY_S]
        if len(live) != len(latched):
            with self._lock:
                live_keys = {icao for icao, _ in live}
                self._latched = {k: v for k, v in self._latched.items()
                                 if k in live_keys}

        # ADS-B tracks render every frame at their live position, faded by
        # seconds-since-last-message (NOT by the sweep).  Snapshot them here.
        import adsb_source as _asrc
        adsb_snap = []
        with self.sim._lock:
            for ac in self.sim._aircraft:
                if not isinstance(ac, _asrc.AdsbAircraft):
                    continue
                if ac.lat is None or ac.lon is None:
                    continue
                age_msg = ac.seconds_since_last_msg()
                if age_msg > _ADSB_FADE_S * 2:      # invisible past 2× fade
                    continue
                # Only draw an "arrow" when we have a heading; otherwise a
                # circle so we don't fake a direction.
                hdg = None
                dac = ac._decoder_ac
                if dac.track is not None or dac.heading is not None:
                    hdg = dac.track if dac.track is not None else dac.heading
                adsb_snap.append((ac.icao, ac.callsign, ac.color, ac.lat, ac.lon,
                                  hdg, age_msg))

        # Fade-step quantised signature — includes both sim-latched and ADS-B.
        blip_sig = (
            tuple(
                (icao, round(snap[1], 5), round(snap[2], 5),
                 None if snap[3] is None else round(snap[3], 1),
                 int((now - snap[0]) * 8))
                for icao, snap in live
            ),
            tuple(
                (icao, round(lat, 5), round(lon, 5),
                 None if hdg is None else round(hdg, 1),
                 int(min(age, 999) * 4))            # 4 fade steps per second
                for icao, callsign, col, lat, lon, hdg, age in adsb_snap
            ),
        )
        if blip_sig == self._blip_sig:
            return
        self._blip_sig = blip_sig

        cv.delete("blip")

        # Sim aircraft — sweep-latched, fade over _BLIP_DECAY_S toward ring grey.
        for icao, snap in live:
            ts, lat, lon, hdg, col, callsign, *_rest = snap
            pt = self._to_xy(lat, lon)
            if pt is None:
                continue
            x, y = pt
            age = now - ts
            faded = ui.blend(col, ui.DIM, age / _BLIP_DECAY_S)
            if hdg is None:
                rad = ui.BLIP_SZ * sf
                cv.create_oval(x - rad, y - rad, x + rad, y + rad,
                               fill=faded, outline="", tags="blip")
            else:
                ui.draw_blip(cv, x, y, math.radians(hdg), faded, sf, tag="blip")
            cv.create_text(x + ui.LBL_DX * sf, y - ui.LBL_DY * sf,
                           text=callsign, fill=faded,
                           font=ui.sfont(ui.PT_MD, sf, bold=True),
                           anchor="w", tags="blip")

        # ADS-B aircraft — live position, fade by seconds-since-last-message.
        for icao, callsign, col, lat, lon, hdg, age_msg in adsb_snap:
            pt = self._to_xy(lat, lon)
            if pt is None:
                continue
            x, y = pt
            faded = ui.blend(col, ui.DIM, min(age_msg / _ADSB_FADE_S, 1.0))
            if hdg is None:
                rad = ui.BLIP_SZ * sf
                cv.create_oval(x - rad, y - rad, x + rad, y + rad,
                               fill=faded, outline="", tags="blip")
            else:
                ui.draw_blip(cv, x, y, math.radians(hdg), faded, sf, tag="blip")
            cv.create_text(x + ui.LBL_DX * sf, y - ui.LBL_DY * sf,
                           text=callsign, fill=faded,
                           font=ui.sfont(ui.PT_MD, sf, bold=True),
                           anchor="w", tags="blip")

    def invalidate_overlay(self):
        """Called by the sim when the view changes (resize/recenter)."""
        self._beam_sig = None
        self._blip_sig = None

    # ── track table + last-reply pane ────────────────────────────────────────

    # Fixed column widths — keep total ≤ panel width so the row fits without wrap.
    _COL_FMT = "{callsign:<8} {sqwk:>5} {rng:>6} {brg:>5} {alt:>6} {age:>4}"

    def _table_row(self, *, callsign, sqwk, rng, brg, alt, age):
        return self._COL_FMT.format(callsign=callsign[:8], sqwk=sqwk,
                                    rng=rng, brg=brg, alt=alt, age=age)

    def _flush_table(self):
        """Rebuild the track table from _latched + _adsb_meta.
        Sim aircraft: age is time since the beam painted them.
        ADS-B aircraft: age is seconds-since-last-ADS-B-message."""
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

        # ADS-B rows: read live position from the wrapper, age from decoder.
        # Also collect the set of ICAOs still live so we can drop stale meta.
        import adsb_source as _asrc
        live_adsb_icaos = set()
        with self.sim._lock:
            for ac in self.sim._aircraft:
                if not isinstance(ac, _asrc.AdsbAircraft):
                    continue
                if ac.lat is None or ac.lon is None:
                    continue
                age = ac.seconds_since_last_msg()
                if age > _ADSB_FADE_S * 2:
                    continue
                live_adsb_icaos.add(ac.icao)
                meta = self._adsb_meta.get(ac.icao)
                if meta is None:
                    # We've never interrogated this ICAO yet — still show it in
                    # the table with sensible placeholders so the operator sees
                    # it before the first sweep hits.
                    code, msa = 0, ac.modes_addr
                    mode = iff.MODE_S_AC
                else:
                    _col, _call, code, _alt, msa, mode, _raw = meta
                brg, rng = _bearing_nm(self.c_lat, self.c_lon, ac.lat, ac.lon)
                entries.append((rng, ac.icao, ac.callsign, code, ac.alt_ft, msa,
                                mode, brg, age))
        # Prune ADS-B meta for aircraft that aren't live anymore so the LAST
        # REPLY pane doesn't linger on stale hex forever.
        with self._lock:
            for stale in [i for i in self._adsb_meta if i not in live_adsb_icaos]:
                self._adsb_meta.pop(stale, None)

        # Drop the live ordering on the user (closest first).
        entries.sort(key=lambda t: t[0])

        # Build the rows
        self._tbl.config(state=tk.NORMAL)
        self._tbl.delete("1.0", tk.END)
        self._row_icaos = []
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
        """Show the most recent raw reply hex for the currently selected ICAO.
        Checks the sim-latched dict first, then the ADS-B meta dict."""
        icao = self._selected_icao
        if icao is None:
            self._v_detail.set("(click an aircraft on the radar or in the table)")
            return
        with self._lock:
            snap = self._latched.get(icao)
            adsb = self._adsb_meta.get(icao) if snap is None else None
        if snap is not None:
            _ts, _lat, _lon, _hdg, _col, callsign, _code, _alt, _msa, mode, raw = snap
        elif adsb is not None:
            _col, callsign, _code, _alt, _msa, mode, raw = adsb
        else:
            self._v_detail.set(f"{icao}: no recent reply")
            return
        head = f"{callsign}  {iff.MODE_NAMES.get(mode, '?')}"
        self._v_detail.set(head + "\n" + iff.format_hex(raw))

    def _on_table_click(self, ev):
        """Click selects that aircraft (also populates the sim's AIRCRAFT panel)."""
        idx = self._tbl.index(f"@{ev.x},{ev.y}")
        line_no = int(idx.split(".")[0])
        if 1 <= line_no <= len(self._row_icaos):
            self._select_by_icao(self._row_icaos[line_no - 1])

    def try_select_blip(self, ev_x, ev_y) -> bool:
        """Hit-test against the latched blip positions; select the nearest.

        Called by the sim's press handler.  Returns True if a blip was hit
        (so the sim doesn't also place a waypoint at the same press location).
        """
        with self._lock:
            entries = [(icao, snap[1], snap[2]) for icao, snap in self._latched.items()]
        best = None
        best_d = ui.HIT_WP * ui.scale_for(self.sim._cw, self.sim._ch) * 3.0
        for icao, lat, lon in entries:
            pt = self._to_xy(lat, lon)
            if pt is None:
                continue
            d = math.hypot(ev_x - pt[0], ev_y - pt[1])
            if d < best_d:
                best_d = d
                best = icao
        if best is None:
            return False
        self._select_by_icao(best)
        return True
