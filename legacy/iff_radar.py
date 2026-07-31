#!/usr/bin/env python3
"""
IFF Radar
=========
Standalone sweeping-antenna interrogator.  This is the *radar side* of the
IFF simulation pair — it has no access to aircraft ground truth and never
reads another process's state directly.  Everything it knows about a target
comes from interrogation replies received over UDP from an `aircraft_sim.py`
process (or any other IFF-equipped target answering on that channel).

Because each mode carries only what real IFF actually delivers, a track's
fields fill in opportunistically as different modes are used: interrogate in
Mode 3/A and only a squawk appears; switch to Mode C and altitude appears
too; only a Mode S Selective interrogation with the callsign register (BDS
2,0) ever reveals a callsign.  Nothing is shown before its mode has actually
been used.

In addition to the interrogation picture, the radar passively receives ADS-B
broadcasts (the same feed path_emulator.py transmits) and fuses those tracks
onto the same display.  ADS-B is never interrogated — it is a broadcast, so
those tracks are drawn as heading-aligned triangles (ADS-B reports track)
with an "A"-prefixed track number, versus the plain circles used for IFF.

Usage
-----
    python iff_radar.py
    python iff_radar.py --centre 51.5,-0.5 --range 150
"""

import argparse
import math
import socket
import threading
import time
import tkinter as tk
from datetime import datetime, timezone

# Superseded by simulator.py and moved into legacy/.  The shared modules this
# program still uses (radar_ui, net_config, adsb_decoder, ...) stayed at the
# repo root, so put the root on the path before importing them.
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))

import adsb_decoder
import iff_interrogation as ii
import iff_protocol as iff
import net_config
import radar_ui as ui
import udp_endpoints as udp


# ── Defaults / limits ─────────────────────────────────────────────────────────

_DEFAULT_RPM  = 15
_DEFAULT_BW   = 3        # degrees
_DEFAULT_PRT  = 30000    # microseconds (30 ms) — real UDP round trips are
                         # now part of the loop, so this is well above the
                         # old loopback-ground-truth default of 1 ms.
_MIN_PRT_S    = 0.001
_BLIP_DECAY_S = 4.0      # IFF latched blip fades toward ring-grey over this long
_ADSB_FADE_S  = 5.0      # passively-received ADS-B track fades over this long

_MODE_LABELS = [
    ("Mode 1",            iff.MODE_1),
    ("Mode 2",            iff.MODE_2),
    ("Mode 3/A",          iff.MODE_3A),
    ("Mode C",            iff.MODE_C),
    ("Mode S All-Call",   iff.MODE_S_AC),
    ("Mode S Selective",  iff.MODE_S_SEL),
]


# ── Radar app ─────────────────────────────────────────────────────────────────

class RadarApp(tk.Tk):
    """Sweeping radar UI: interrogation panel, PPI with beam wedge + latched
    blips, track table, and last-reply hex pane.

    Two independent UDP links:
      - the aircraft-interrogation channel: this radar's own interrogation
        packets out, per-target replies in.  Private to the simulation pair.
      - the external channel: an unchanged 26-byte control message that can
        override this radar's active mode, and the outbound consolidated
        reply block in the pre-existing external format.
    """

    def __init__(self, c_lat, c_lon, rng):
        super().__init__()
        self.title("IFF Radar")
        self.configure(bg=ui.PANEL)
        self.resizable(True, True)
        self.minsize(round(560 * ui.SCALE), round(420 * ui.SCALE))

        self.c_lat, self.c_lon, self.rng = c_lat, c_lon, rng
        self._cw = self._ch = ui.CANVAS_SZ
        self._fullscreen = False
        self._bg_sig   = None
        self._beam_sig = None
        self._blip_sig = None

        # Sweep state.  Azimuth is derived from the wall clock + RPM (see
        # _azimuth_now) rather than integrated per Tk frame, so the TX thread
        # (which can run much faster than the 20 Hz redraw) always samples
        # the current antenna angle instead of a stale cached value.
        self._sweep_anchor_t  = time.monotonic()
        self._sweep_anchor_az = 0.0
        self._sweep_rpm       = _DEFAULT_RPM

        # prt_no bookkeeping + per-PRT reply batching for the external
        # consolidated reply-block-out.  A reply is added to the batch only
        # if its prt_no matches the interrogation currently in flight, so by
        # construction the batch is exactly "who replied to that sweep" —
        # the aircraft side already did the beam/capability/address gating.
        self._prt_no = 0
        self._batch_lock = threading.Lock()
        self._current_batch: list = []

        # Tracks accumulate fields opportunistically as different modes are
        # used; a field only appears once its mode has actually delivered it.
        # icao_env (6-hex str) -> {'m1':, 'm2':, 'sqwk':, 'alt_ft':,
        #   'modes_addr':, 'call':, 'color':, 'last_lat':, 'last_lon':,
        #   'last_ts':, 'last_mode':, 'last_raw': bytes}
        self._tracks: dict[str, dict] = {}
        self._track_no_by_icao: dict[str, int] = {}
        self._next_track_no = 1
        # Only Mode S replies reveal a 24-bit address — selective addressing
        # can only target something already learned this way.
        self._known_addrs: dict[str, int] = {}
        self._tracks_lock = threading.Lock()
        self._target_menu_map: dict[str, int] = {}   # dropdown label -> addr
        self._target_menu_dirty = True

        # Passive ADS-B surveillance: a completely separate data source from
        # IFF interrogation.  We just listen to the broadcast (whatever
        # path_emulator or a real feed sends on the ADS-B multicast group) and
        # display it fused with the IFF picture.  ADS-B is never interrogated.
        self._adsb_fleet: dict[str, "adsb_decoder.Aircraft"] = {}
        self._adsb_lock = threading.Lock()
        self._adsb_track_no_by_icao: dict[str, int] = {}
        self._next_adsb_track_no = 1

        self._selected_icao = None
        self._table_dirty = False
        self._table_last  = 0.0
        self._row_icaos: list[str] = []

        # Tk vars for the interrogation panel.
        self._v_mode         = tk.StringVar(value=_MODE_LABELS[2][0])
        self._v_target       = tk.StringVar(value="(no known address)")
        self._v_use_external = tk.BooleanVar(value=False)
        self._v_rpm          = tk.IntVar(value=_DEFAULT_RPM)
        self._v_bw           = tk.DoubleVar(value=_DEFAULT_BW)
        self._v_prt          = tk.IntVar(value=_DEFAULT_PRT)

        # External-mode state, populated by the (unchanged) 26-byte control
        # message receiver.
        self._external_mode:   int | None = None
        self._external_target: int | None = None
        self._ext_status_snap = "(no messages yet)"

        # Plain-Python mirrors of the Tk vars.  The TX thread must never call
        # tk.Variable.get() itself — Python 3.13+ requires that run on the
        # Tk main thread — so tick() refreshes these once per frame instead.
        self._rpm_snap          = _DEFAULT_RPM
        self._bw_snap           = float(_DEFAULT_BW)
        self._prt_s_snap        = _DEFAULT_PRT * 1e-6
        self._use_external_snap = False
        self._mode_label_snap   = _MODE_LABELS[2][0]
        self._target_addr_snap: int | None = None

        self._build_ui()
        self.bind("<F11>",    self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)

        cfg = net_config.load()
        self._cfg = cfg

        # Aircraft-interrogation channel (private to the simulation pair).
        self._int_send_sock, self._int_send_dst = udp.open_send(
            cfg["ac_channel_int_host"], cfg["ac_channel_int_port"],
            cfg["ac_channel_int_transport"], cfg["ac_channel_int_iface"])
        self._rep_recv_sock = udp.open_recv(
            cfg["ac_channel_rep_host"], cfg["ac_channel_rep_port"],
            cfg["ac_channel_rep_transport"], cfg["ac_channel_rep_iface"])
        self._rep_recv_sock.settimeout(0.5)

        # External control-message input + consolidated reply-block output —
        # unchanged formats/endpoints from before the aircraft/radar split.
        self._reply_sock, self._reply_dst = udp.open_send(
            cfg["iff_reply_host"], cfg["iff_reply_port"],
            cfg["iff_reply_transport"], cfg["iff_reply_iface"])
        self._interrogation_rx = ii.Receiver(
            cfg["iff_interrogation_host"], cfg["iff_interrogation_port"],
            cfg["iff_interrogation_transport"], cfg["iff_interrogation_iface"],
            on_message=self._on_external_interrogation)
        self._interrogation_rx.start()

        # Passive ADS-B receiver: join the ADS-B multicast group and decode
        # broadcasts into a private fleet for display alongside IFF tracks.
        self._adsb_sock = None
        try:
            self._adsb_sock = udp.open_recv(cfg["group"], cfg["port"],
                                            "multicast", cfg["iface"])
            self._adsb_sock.settimeout(0.5)
        except OSError as e:
            print(f"[iff_radar] ADS-B receive socket failed: {e}")

        self._stop = threading.Event()
        threading.Thread(target=self._tx_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()
        if self._adsb_sock is not None:
            threading.Thread(target=self._adsb_rx_loop, daemon=True).start()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._loop()

    def _on_close(self):
        self._stop.set()
        self._interrogation_rx.stop()
        for s in (self._int_send_sock, self._rep_recv_sock, self._reply_sock,
                  self._adsb_sock):
            if s is None:
                continue
            try:
                s.close()
            except OSError:
                pass
        self.destroy()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.cv = tk.Canvas(self, width=ui.CANVAS_SZ, height=ui.CANVAS_SZ,
                            bg=ui.BG, highlightthickness=0, cursor="crosshair")
        self.cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.cv.bind("<Configure>", self._on_resize)
        self.cv.bind("<Button-1>",  self._on_canvas_click)

        p = ui.make_panel(self, side=tk.RIGHT)
        self._panel = p

        # ── INTERROGATION ──
        tk.Frame(p, bg=ui.PANEL, height=round(10 * ui.SCALE)).pack()
        tk.Label(p, text="INTERROGATION", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)

        labels = [lbl for lbl, _ in _MODE_LABELS]
        mf = tk.Frame(p, bg=ui.PANEL)
        mf.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        tk.Label(mf, text="mode", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        om = tk.OptionMenu(mf, self._v_mode, *labels)
        om.config(bg=ui.ENTRY, fg=ui.FG, activebackground=ui.BTN_ACT,
                  font=ui.F_MD, relief=tk.FLAT, bd=0, highlightthickness=0)
        om["menu"].config(bg=ui.ENTRY, fg=ui.FG, font=ui.F_MD)
        om.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # External-mode toggle: when on, mode comes from incoming 26-byte
        # control messages; the dropdown becomes advisory.
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

        # Selective-target dropdown: only ICAOs whose 24-bit address has
        # actually been learned via a prior Mode S reply appear here — you
        # cannot selectively address something whose address you don't know.
        tf = tk.Frame(p, bg=ui.PANEL)
        tf.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        tk.Label(tf, text="target", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        self._target_om = tk.OptionMenu(tf, self._v_target, "(no known address)")
        self._target_om.config(bg=ui.ENTRY, fg=ui.FG, activebackground=ui.BTN_ACT,
                               font=ui.F_MD, relief=tk.FLAT, bd=0, highlightthickness=0)
        self._target_om["menu"].config(bg=ui.ENTRY, fg=ui.FG, font=ui.F_MD)
        self._target_om.pack(side=tk.LEFT, fill=tk.X, expand=True)

        ui.slider_row(p, "rpm",    self._v_rpm, 5,    30)
        ui.slider_row(p, "bw °",   self._v_bw,  1.0,  10.0,   resolution=0.5)
        ui.slider_row(p, "prt µs", self._v_prt, 5000, 200000, resolution=1000)

        # ── TRACKS ──
        ui.sep(p)
        tk.Label(p, text="TRACKS", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)

        table_frame = tk.Frame(p, bg=ui.PANEL)
        table_frame.pack(fill=tk.BOTH, expand=True, padx=ui.PAD, pady=(0, ui.PAD2))
        self._tbl_hdr = tk.Label(table_frame, bg=ui.PANEL, fg=ui.FG_DIM,
                                 font=ui.F_SM, anchor="w", justify=tk.LEFT,
                                 text=self._table_row(trk="TRK", call="CALL", addr="ADDR",
                                                      sqwk="SQWK", m1="M1", m2="M2",
                                                      alt="ALT", rng="RNG", brg="BRG",
                                                      age="AGE"))
        self._tbl_hdr.pack(fill=tk.X)

        self._tbl = tk.Text(table_frame, bg=ui.ENTRY, fg=ui.FG, font=ui.F_SM,
                            relief=tk.FLAT, bd=0, wrap=tk.NONE,
                            cursor="hand2", highlightthickness=0,
                            state=tk.DISABLED, height=12)
        self._tbl.pack(fill=tk.BOTH, expand=True)
        self._tbl.bind("<Button-1>", self._on_table_click)
        self._tbl.tag_configure("row", foreground=ui.FG)
        self._tbl.tag_configure("sel", background="#2a2a2a", foreground="#ffffff")

        # ── LAST REPLY ──
        ui.sep(p)
        tk.Label(p, text="LAST REPLY", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)
        self._v_detail = tk.StringVar(value="(click a track on the radar or in the table)")
        tk.Label(p, textvariable=self._v_detail, bg=ui.PANEL, fg="#888888",
                 font=ui.F_SM, anchor="w", justify=tk.LEFT,
                 wraplength=ui.PANEL_W - 2 * ui.PAD
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

    # ── sweep azimuth ─────────────────────────────────────────────────────────

    def _azimuth_now(self) -> float:
        rpm = self._rpm_snap
        if rpm != self._sweep_rpm:
            self._sweep_anchor_az = self._azimuth_at(time.monotonic(), self._sweep_rpm)
            self._sweep_anchor_t  = time.monotonic()
            self._sweep_rpm       = rpm
        return self._azimuth_at(time.monotonic(), rpm)

    def _azimuth_at(self, t: float, rpm: float) -> float:
        return (self._sweep_anchor_az +
                (t - self._sweep_anchor_t) * rpm * 6.0) % 360.0

    # ── mode selection ────────────────────────────────────────────────────────

    def _current_mode(self) -> int:
        if self._use_external_snap and self._external_mode is not None:
            return self._external_mode
        for lbl, code in _MODE_LABELS:
            if lbl == self._mode_label_snap:
                return code
        return iff.MODE_3A

    def _external_selective_addr(self):
        if self._use_external_snap:
            return self._external_target
        return None

    def _on_external_interrogation(self, msg):
        """Runs on the ii.Receiver thread — writes only plain attributes."""
        self._external_mode = msg.mode
        if msg.mode in (iff.MODE_S_AC, iff.MODE_S_SEL):
            self._external_target = msg.modes_addr
        else:
            self._external_target = None
        mode_name = iff.MODE_NAMES.get(msg.mode, f"?{msg.mode}")
        addr_s = f" {msg.modes_addr:06X}" if msg.modes_addr is not None else ""
        self._ext_status_snap = f"{mode_name}{addr_s}"

    def _refresh_target_menu(self):
        if not self._target_menu_dirty:
            return
        self._target_menu_dirty = False
        menu = self._target_om["menu"]
        with self._tracks_lock:
            entries = list(self._known_addrs.items())
        menu.delete(0, "end")
        if not entries:
            menu.add_command(label="(no known address)",
                             command=lambda: self._v_target.set("(no known address)"))
            self._v_target.set("(no known address)")
            self._target_menu_map = {}
            return
        cur = self._v_target.get()
        self._target_menu_map = {}
        for icao, addr in entries:
            trk_no = self._track_no_by_icao.get(icao, 0)
            label = f"TRK{trk_no:03d}  {addr:06X}"
            self._target_menu_map[label] = addr
            menu.add_command(label=label, command=lambda l=label: self._v_target.set(l))
        if cur not in self._target_menu_map:
            self._v_target.set(next(iter(self._target_menu_map)))

    # ── TX loop (background thread): send interrogations, flush batches ─────

    def _tx_loop(self):
        """Send one interrogation per PRT.  Touches no Tk APIs — all slider
        values come from the plain-Python snapshots refreshed by _loop() on
        the Tk main thread."""
        while not self._stop.is_set():
            prt_s = max(self._prt_s_snap, _MIN_PRT_S)
            time.sleep(prt_s)

            # Flush whatever the RX thread batched for the PREVIOUS PRT
            # before sending the next interrogation.  Every reply in the
            # batch already passed the aircraft-side beam/capability/address
            # gate, so the batch is exactly the external reply-block's target
            # list — no re-filtering needed here.
            with self._batch_lock:
                batch = self._current_batch
                self._current_batch = []
            if batch and hasattr(self, "_last_sent_mode"):
                raw = iff.build_reply(prt_no=self._prt_no, azimuth_deg=self._last_sent_az,
                                      mode=self._last_sent_mode, targets=batch)
                try:
                    self._reply_sock.sendto(raw, self._reply_dst)
                except OSError:
                    pass

            mode = self._current_mode()
            bw   = self._bw_snap
            az   = self._azimuth_now()

            sel_addr = 0
            bds_reg  = 0
            if mode == iff.MODE_S_SEL:
                sel_addr = (self._external_selective_addr() or
                            self._target_addr_snap or 0)
                bds_reg = iff.BDS_CALLSIGN

            self._prt_no = (self._prt_no + 1) & 0xFFFF
            self._last_sent_mode = mode
            self._last_sent_az   = az
            pkt = iff.pack_interrogation(mode=mode, prt_no=self._prt_no,
                                        radar_lat=self.c_lat, radar_lon=self.c_lon,
                                        beam_az_deg=az, beam_bw_deg=bw,
                                        selective_addr=sel_addr, bds_reg=bds_reg)
            try:
                self._int_send_sock.sendto(pkt, self._int_send_dst)
            except OSError:
                pass

    # ── RX loop (background thread): receive per-target replies ─────────────

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, _addr = self._rep_recv_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                d = iff.decode_target_reply(data)
            except Exception:
                continue
            self._on_target_reply(d, data)

    def _on_target_reply(self, d: dict, raw_pkt: bytes):
        icao_env = f"{d['icao']:06X}"
        mode = d["mode"]
        now  = time.monotonic()

        with self._tracks_lock:
            if icao_env not in self._track_no_by_icao:
                self._track_no_by_icao[icao_env] = self._next_track_no
                self._next_track_no += 1
            trk = self._tracks.setdefault(icao_env, {})
            trk["last_lat"]  = d["src_lat"]
            trk["last_lon"]  = d["src_lon"]
            trk["last_ts"]   = now
            trk["last_mode"] = mode
            trk["last_raw"]  = raw_pkt
            trk.setdefault("color", ui.random_color())

            # Only the field this specific mode delivers is ever set — a
            # track never shows data it hasn't actually received.
            if mode == iff.MODE_1:
                trk["m1"] = d["mission_code"]
            elif mode == iff.MODE_2:
                trk["m2"] = d["unit_code"]
            elif mode == iff.MODE_3A:
                trk["sqwk"] = d["squawk"]
            elif mode == iff.MODE_C:
                trk["alt_ft"] = d["alt_ft"]
            elif mode == iff.MODE_S_AC:
                trk["modes_addr"] = d["modes_addr"]
                self._known_addrs[icao_env] = d["modes_addr"]
                self._target_menu_dirty = True
            elif mode == iff.MODE_S_SEL:
                trk["modes_addr"] = d["modes_addr"]
                self._known_addrs[icao_env] = d["modes_addr"]
                self._target_menu_dirty = True
                if "callsign" in d:
                    trk["call"] = d["callsign"]

            self._table_dirty = True

        # Batch for the outbound consolidated reply block, only if this
        # reply belongs to the interrogation currently in flight.
        if d["prt_no"] == self._prt_no:
            code = 0
            if mode == iff.MODE_1:
                code = d["mission_code"]
            elif mode == iff.MODE_2:
                code = d["unit_code"]
            elif mode == iff.MODE_3A:
                code = d["squawk"]
            elif mode == iff.MODE_C:
                code = iff.encode_mode_c(d["alt_ft"])
            _brg, rng = iff.bearing_range_nm(self.c_lat, self.c_lon,
                                             d["src_lat"], d["src_lon"])
            rec = iff.TargetRecord(range_nm=rng, code=code,
                                   modes_addr=d.get("modes_addr", 0))
            with self._batch_lock:
                self._current_batch.append(rec)

    # ── ADS-B RX loop (background thread): passive broadcast reception ───────

    def _adsb_rx_loop(self):
        buf = ""
        while not self._stop.is_set():
            try:
                data, _addr = self._adsb_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            buf += data.decode("ascii", errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    with self._adsb_lock:
                        adsb_decoder.decode_message(line, self._adsb_fleet)
                        icao = line.strip("*;")[2:8].upper() if len(line) >= 9 else None
                        if icao and icao not in self._adsb_track_no_by_icao:
                            self._adsb_track_no_by_icao[icao] = self._next_adsb_track_no
                            self._next_adsb_track_no += 1
                except Exception:
                    continue
            self._table_dirty = True

    def _adsb_snapshot(self):
        """Snapshot displayable ADS-B tracks: list of dicts.  ADS-B position
        may be unresolved (CPR pair pending) — those are skipped."""
        now_utc = datetime.now(timezone.utc)
        out = []
        with self._adsb_lock:
            for icao, ac in self._adsb_fleet.items():
                if ac.lat is None or ac.lon is None:
                    continue
                age = ((now_utc - ac.last_seen).total_seconds()
                       if ac.last_seen is not None else 1e9)
                hdg = ac.track if ac.track is not None else ac.heading
                out.append({
                    "icao": icao,
                    "trk_no": self._adsb_track_no_by_icao.get(icao, 0),
                    "lat": ac.lat, "lon": ac.lon, "hdg": hdg,
                    "callsign": (ac.callsign or "").strip(),
                    "alt_ft": ac.altitude,
                    "age": age,
                })
        return out

    # ── render loop (Tk main thread) ──────────────────────────────────────────

    def _loop(self):
        if self._stop.is_set():
            return
        try:
            self._rpm_snap          = self._v_rpm.get()
            self._bw_snap           = float(self._v_bw.get())
            self._prt_s_snap        = self._v_prt.get() * 1e-6
            self._use_external_snap = self._v_use_external.get()
            self._mode_label_snap   = self._v_mode.get()
            self._target_addr_snap  = self._target_menu_map.get(self._v_target.get())
            if self._v_ext_status.get() != self._ext_status_snap:
                self._v_ext_status.set(self._ext_status_snap)
        except Exception:
            pass

        self._draw()

        now = time.monotonic()
        if self._table_dirty or (now - self._table_last) >= 0.2:
            self._flush_table()
            self._table_last = now

        self.after(50, self._loop)

    # ── draw ──────────────────────────────────────────────────────────────────

    def _view_sig(self):
        return (round(self.c_lat, 6), round(self.c_lon, 6), round(self.rng, 3),
                self._cw, self._ch)

    def _draw(self):
        cv = self.cv
        cx, cy, r = ui.geom(self._cw, self._ch)
        sf = ui.scale_for(self._cw, self._ch)

        sig = self._view_sig()
        if sig != self._bg_sig:
            cv.delete("bg")
            ui.draw_radar_frame(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                sf, tag="bg")
            ui.draw_latlon_grid(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                tag="bg")
            self._bg_sig = sig
            self._blip_sig = None

        # Beam wedge — one pie slice centred on antenna azimuth.
        bw = self._bw_snap
        az = self._azimuth_now()
        beam_sig = (round(az, 1), round(bw, 2), self._cw, self._ch)
        if beam_sig != self._beam_sig:
            cv.delete("beam")
            tk_start = (90.0 - az - bw / 2.0) % 360.0
            cv.create_arc(cx - r, cy - r, cx + r, cy + r,
                          start=tk_start, extent=bw,
                          fill="#1a1a1a", outline="", style="pieslice", tags="beam")
            self._beam_sig = beam_sig

        # Two blip populations on one display:
        #   IFF (interrogated): plain circles.  Our modes deliver position but
        #     never heading, so no arrow.  Fade from the last reply over
        #     _BLIP_DECAY_S.
        #   ADS-B (passively received): triangles pointing along track, since
        #     ADS-B does report heading.  Fade by message age over _ADSB_FADE_S.
        now = time.monotonic()
        with self._tracks_lock:
            stale = [icao for icao, trk in self._tracks.items()
                    if now - trk.get("last_ts", 0) > _BLIP_DECAY_S]
            for icao in stale:
                del self._tracks[icao]
            iff_live = [(icao, dict(trk)) for icao, trk in self._tracks.items()]

        adsb_live = [t for t in self._adsb_snapshot() if t["age"] <= _ADSB_FADE_S]

        blip_sig = (
            tuple((icao, round(trk["last_lat"], 5), round(trk["last_lon"], 5),
                   int((now - trk["last_ts"]) * 8))
                  for icao, trk in iff_live),
            tuple((t["icao"], round(t["lat"], 5), round(t["lon"], 5),
                   None if t["hdg"] is None else round(t["hdg"], 1),
                   int(t["age"] * 4))
                  for t in adsb_live),
        )
        if blip_sig == self._blip_sig:
            return
        self._blip_sig = blip_sig

        cv.delete("blip")
        rad = ui.BLIP_SZ * sf

        # IFF circles.
        for icao, trk in iff_live:
            pt = self._to_xy(trk["last_lat"], trk["last_lon"])
            if pt is None:
                continue
            x, y = pt
            faded = ui.blend(trk.get("color", ui.FG), ui.DIM,
                             min((now - trk["last_ts"]) / _BLIP_DECAY_S, 1.0))
            cv.create_oval(x - rad, y - rad, x + rad, y + rad,
                           fill=faded, outline="", tags="blip")
            label = trk.get("call") or f"TRK{self._track_no_by_icao.get(icao, 0):03d}"
            cv.create_text(x + ui.LBL_DX * sf, y - ui.LBL_DY * sf,
                           text=label, fill=faded,
                           font=ui.sfont(ui.PT_MD, sf, bold=True),
                           anchor="w", tags="blip")

        # ADS-B triangles.
        for t in adsb_live:
            pt = self._to_xy(t["lat"], t["lon"])
            if pt is None:
                continue
            x, y = pt
            faded = ui.blend(self._adsb_color(t["icao"]), ui.DIM,
                             min(t["age"] / _ADSB_FADE_S, 1.0))
            if t["hdg"] is None:
                cv.create_oval(x - rad, y - rad, x + rad, y + rad,
                               fill=faded, outline="", tags="blip")
            else:
                ui.draw_blip(cv, x, y, math.radians(t["hdg"]), faded, sf, tag="blip")
            label = t["callsign"] or f"A{t['trk_no']:02d}"
            cv.create_text(x + ui.LBL_DX * sf, y - ui.LBL_DY * sf,
                           text=label, fill=faded,
                           font=ui.sfont(ui.PT_MD, sf, bold=True),
                           anchor="w", tags="blip")

    def _adsb_color(self, icao):
        """Stable per-ICAO colour for ADS-B tracks."""
        cache = getattr(self, "_adsb_colors", None)
        if cache is None:
            cache = self._adsb_colors = {}
        col = cache.get(icao)
        if col is None:
            col = cache[icao] = ui.random_color()
        return col

    # ── selection ─────────────────────────────────────────────────────────────

    def _on_canvas_click(self, ev):
        with self._tracks_lock:
            entries = [(icao, trk["last_lat"], trk["last_lon"])
                       for icao, trk in self._tracks.items()]
        # ADS-B tracks are keyed with an "A:" prefix so their ICAO can never
        # collide with an IFF track's sim-envelope ICAO in the selection state.
        for t in self._adsb_snapshot():
            entries.append((f"A:{t['icao']}", t["lat"], t["lon"]))
        best, best_d = None, ui.HIT_WP * ui.scale_for(self._cw, self._ch) * 3.0
        for key, lat, lon in entries:
            pt = self._to_xy(lat, lon)
            if pt is None:
                continue
            d = math.hypot(ev.x - pt[0], ev.y - pt[1])
            if d < best_d:
                best_d = d
                best = key
        self._selected_icao = best
        self._table_dirty = True

    def _on_table_click(self, ev):
        idx = self._tbl.index(f"@{ev.x},{ev.y}")
        line_no = int(idx.split(".")[0])
        if 1 <= line_no <= len(self._row_icaos):
            self._selected_icao = self._row_icaos[line_no - 1]
            self._table_dirty = True

    # ── track table + last-reply pane ────────────────────────────────────────

    _COL_FMT = ("{trk:>3} {call:<8} {addr:>6} {sqwk:>4} {m1:>2} {m2:>4} "
               "{alt:>5} {rng:>5} {brg:>3} {age:>4}")

    def _table_row(self, *, trk, call, addr, sqwk, m1, m2, alt, rng, brg, age):
        return self._COL_FMT.format(trk=trk, call=call[:8], addr=addr, sqwk=sqwk,
                                    m1=m1, m2=m2, alt=alt, rng=rng, brg=brg, age=age)

    def _flush_table(self):
        now = time.monotonic()
        # rows: (rng, key, tag, dict-of-columns)
        rows = []
        with self._tracks_lock:
            for icao, trk in self._tracks.items():
                age = now - trk.get("last_ts", now)
                if age > _BLIP_DECAY_S:
                    continue
                brg, rng = iff.bearing_range_nm(self.c_lat, self.c_lon,
                                                trk["last_lat"], trk["last_lon"])
                rows.append((rng, icao, {
                    "trk":  f"{self._track_no_by_icao.get(icao, 0):03d}",
                    "call": trk.get("call") or "—",
                    "addr": f"{trk['modes_addr']:06X}" if "modes_addr" in trk else "—",
                    "sqwk": f"{trk['sqwk']:04o}"        if "sqwk"       in trk else "—",
                    "m1":   f"{trk['m1']:02o}"          if "m1"         in trk else "—",
                    "m2":   f"{trk['m2']:04o}"          if "m2"         in trk else "—",
                    "alt":  f"FL{trk['alt_ft']//100:03d}" if "alt_ft"   in trk else "—",
                    "rng":  f"{rng:5.1f}", "brg": f"{brg:3.0f}", "age": f"{age:4.1f}",
                }))
            self._table_dirty = False

        # ADS-B rows — passive source; fills CALL/ADDR/ALT/RNG/BRG only.  The
        # IFF-only columns (SQWK/M1/M2) stay "—" because ADS-B carries none of
        # them.  Keyed "A:<icao>" and shown with an A-prefixed track number.
        for t in self._adsb_snapshot():
            if t["age"] > _ADSB_FADE_S:
                continue
            brg, rng = iff.bearing_range_nm(self.c_lat, self.c_lon, t["lat"], t["lon"])
            rows.append((rng, f"A:{t['icao']}", {
                "trk":  f"A{t['trk_no']:02d}",
                "call": t["callsign"] or "—",
                "addr": t["icao"],
                "sqwk": "—", "m1": "—", "m2": "—",
                "alt":  f"FL{t['alt_ft']//100:03d}" if t["alt_ft"] is not None else "—",
                "rng":  f"{rng:5.1f}", "brg": f"{brg:3.0f}", "age": f"{t['age']:4.1f}",
            }))

        rows.sort(key=lambda t: t[0])

        self._tbl.config(state=tk.NORMAL)
        self._tbl.delete("1.0", tk.END)
        self._row_icaos = []
        for rng, key, col in rows:
            row = self._table_row(**col)
            tag = "sel" if key == self._selected_icao else "row"
            self._tbl.insert(tk.END, row + "\n", tag)
            self._row_icaos.append(key)
        if not rows:
            self._tbl.insert(tk.END, "  (no tracks)\n", "row")
        self._tbl.config(state=tk.DISABLED)

        self._refresh_target_menu()
        self._refresh_detail()

    def _refresh_detail(self):
        key = self._selected_icao
        if key is None:
            self._v_detail.set("(click a track on the radar or in the table)")
            return
        # ADS-B selection: no interrogation reply exists — it's a broadcast, so
        # show the decoded state instead of a reply hex dump.
        if isinstance(key, str) and key.startswith("A:"):
            icao = key[2:]
            with self._adsb_lock:
                ac = self._adsb_fleet.get(icao)
                trk_no = self._adsb_track_no_by_icao.get(icao, 0)
                if ac is None:
                    self._v_detail.set(f"A{trk_no:02d}: no ADS-B data")
                    return
                call = (ac.callsign or "").strip() or "—"
                alt  = f"FL{ac.altitude//100:03d}" if ac.altitude is not None else "—"
                pos  = (f"{ac.lat:+.4f} {ac.lon:+.4f}"
                        if ac.lat is not None else "—")
            self._v_detail.set(
                f"A{trk_no:02d}  ADS-B  {icao}\n"
                f"call {call}   {alt}\npos  {pos}")
            return
        with self._tracks_lock:
            trk = self._tracks.get(key)
        if trk is None or "last_raw" not in trk:
            self._v_detail.set(f"TRK{self._track_no_by_icao.get(key, 0):03d}: no recent reply")
            return
        mode_name = iff.MODE_NAMES.get(trk.get("last_mode"), "?")
        head = f"TRK{self._track_no_by_icao.get(key, 0):03d}  {mode_name}"
        self._v_detail.set(head + "\n" + iff.format_hex(trk["last_raw"]))


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="IFF radar (sweeping interrogator)",
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__)
    p.add_argument("--centre", metavar="LAT,LON", default=None)
    p.add_argument("--range",  type=float, default=200.0)
    args = p.parse_args()
    lat, lon = 51.477, -0.461
    if args.centre:
        lat, lon = map(float, args.centre.split(","))
    RadarApp(lat, lon, args.range).mainloop()


if __name__ == "__main__":
    main()
