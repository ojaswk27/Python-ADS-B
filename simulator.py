#!/usr/bin/env python3
"""
IFF Radar + Aircraft Simulator (single-process)
================================================
Combines the radar interrogator, the aircraft waypoint editor, and the ADS-B
emitter into one self-contained window.  Every aircraft simultaneously:
  • follows a waypoint path
  • emits ADS-B broadcasts continuously
  • answers any mode of IFF interrogation

No UDP — everything runs in-process on the Tk main thread.

Controls
--------
    Left-click canvas   add a single waypoint (auto-creates aircraft if none)
    Click-drag canvas   draw a freehand path (samples points as you drag)
    Drag waypoint dot   reposition that waypoint
    Right-click dot     delete waypoint  /  R-click empty: toggle crosshair
    loop checkbox       close the path into a loop (off = open path)
    Panel list          select aircraft
    address / callsign  edit ICAO + callsign (Enter or click away to apply)
    IFF fields          Mode 1 (5-bit), Mode 2 / 3-A (12-bit) squawks; Mode S addr
    capability flags    M1 / M2 / M3-A / MC / MS transponder on/off
    alt / speed sliders update altitude / speed live
    F11 / Esc           toggle / leave fullscreen

Usage
-----
    python simulator.py
    python simulator.py --centre 51.5,-0.5 --range 150
    python simulator.py --declination 1.5
"""

import argparse
import math
import os
import random
import time
import tkinter as tk

import channel
import iff_protocol as iff
import pseudo1090
import radar_ui as ui
from aircraft_emulator import build_identification, build_position, build_velocity
from receiver import Receiver


# ── Constants ─────────────────────────────────────────────────────────────────

_PANEL_W    = 280 * ui.SCALE
_REPLY_W    = 250 * ui.SCALE
_DEFAULT_RPM  = 15
_DEFAULT_BW   = 3        # degrees
_DEFAULT_PRT  = 30000    # microseconds
_BLIP_DECAY_S = 4.0

_FRAME_MS = 50           # Tk frame period
_MIN_PRT_S = 5000e-6     # matches the PRT slider floor
_MAX_PRT_PER_FRAME = 256

# Below this many interrogations per beam dwell, detection gets unreliable —
# flagged in the panel rather than hidden by fudging the beam width.
_DWELL_MIN_HITS = 3.0
_DWELL_LOW_COL  = "#cc3333"

# Mode S: an aircraft that has answered an all-call is locked out (II/SI) and
# stops answering all-calls for this long, so acquisition happens once per
# aircraft rather than every PRT forever.
_LOCKOUT_S = 18.0

# Ground-truth debug overlay: deliberately drab so it can never be mistaken
# for a plot.
_TRUTH_COL = "#334033"

# 1090 MHz airtime log: how many lines to keep.
_LOG_MAX = 400
_LOG_LOST_COL = "#7a4a3a"

# Reported-heading turn rate, deg/s.  0 = instant, which is the default: the
# aircraft reports the segment course as flown, so a waypoint corner produces a
# step change in the ADS-B velocity stream.  That is deliberate — this is a test
# tool, and feeding a receiver an impossible turn is a thing you may want to do.
#
# Raise it to smooth the reported heading instead.  Note the trade: while the
# limiter is catching up, reported heading disagrees with the actual direction
# of motion, because position still follows the waypoint polyline exactly
# rather than flying a lead-turn arc.  A 90 deg corner at 450 kt and 3 deg/s
# disagrees by up to 90 deg for 30 s.
_DEFAULT_TURN_RATE = 0.0
_TURN_RATE_MAX     = 9.0

# Default climb/descent rate.  The altitude slider sets a TARGET the aircraft
# flies toward, so the reported vertical rate is real instead of always zero.
_CLIMB_FPM = 1800.0

# DO-260B nominal squitter rates, as (min, max) seconds between messages.
_ADSB_INTERVAL = {
    "POS":   (0.4, 0.6),     # airborne position, ~2 Hz
    "VEL":   (0.4, 0.6),     # airborne velocity, ~2 Hz
    "IDENT": (4.8, 5.2),     # aircraft identification, ~every 5 s
}

# SPI (Special Position Identification) — the "ident" pulse, held this long.
_SPI_S = 18.0

_EMERG_COL = "#e0b040"

# Radar-level Mode 2 unit code, inherited by new aircraft.
_DEFAULT_UNIT_CODE = 0o1234


_MODE_LABELS = [
    ("Mode 1",            iff.MODE_1),
    ("Mode 2",            iff.MODE_2),
    ("Mode 3/A",          iff.MODE_3A),
    ("Mode C",            iff.MODE_C),
    ("Mode S All-Call",   iff.MODE_S_AC),
    ("Mode S Selective",  iff.MODE_S_SEL),
]


# Cursor crosshair arm length (px, before scale_for)
_CURSOR_ARM_PX = 22 * ui.SCALE

# Freehand drawing: waypoint spacing in px, at scale_for() == 1.0.  Scaled by
# the live canvas scale at use, so drawing and hit-testing agree after a resize.
_DRAW_SPACING_PX = 9


# Per-field age limit.  Every track field carries its own timestamp and ages
# out on its own schedule: a Mode 1 reply must not keep a 30-second-old squawk
# on screen just because it refreshed the track as a whole.
_FIELD_STALE_S = _BLIP_DECAY_S


def _fld(d, key, now, limit=_FIELD_STALE_S):
    """Read a (value, timestamp) field.  Returns the value while it is younger
    than `limit`, else None, so stale fields dash out of the display
    independently of one another."""
    ent = d.get(key)
    if ent is None:
        return None
    val, ts = ent
    return val if (now - ts) <= limit else None


def _fld_age(d, key, now):
    """Age in seconds of a (value, timestamp) field, or None if never set."""
    ent = d.get(key)
    return None if ent is None else now - ent[1]


# Both 1090 formats deliver the same kinds of value, so the display takes the
# freshest of the two rather than favouring either.  Returns (value, ts, which)
# or None.  Keys beginning with "_" are internal to a record, never fields.
_RX_RECORDS = ("adsb", "pseudo")


def _rx_field(trk, key, now, limit=_FIELD_STALE_S):
    best = None
    for which in _RX_RECORDS:
        ent = (trk.get(which) or {}).get(key)
        if not isinstance(ent, tuple) or len(ent) != 2:
            continue
        val, ts = ent
        if (now - ts) > limit:
            continue
        if best is None or ts > best[1]:
            best = (val, ts, which)
    return best


_TRANSITION_ALT_FT = 18000

def fmt_alt(ft):
    if ft is None:
        return "—"
    if ft >= _TRANSITION_ALT_FT:
        return f"FL{int(round(ft / 100.0)):03d}"
    return f"{int(round(ft / 100.0)) * 100:d}"


# ── Aircraft model ────────────────────────────────────────────────────────────

_ctr = [0]

# Allocate out of the UK block (0x400000-0x43FFFF) rather than 0xFFxxxx, which
# is not assigned to any state.  0x000000 and 0xFFFFFF are both illegal.
_ADDR_BLOCK_BASE = 0x400000
_ADDR_BLOCK_SIZE = 0x040000


def _new_id():
    _ctr[0] += 1
    addr = _ADDR_BLOCK_BASE + (_ctr[0] % _ADDR_BLOCK_SIZE)
    while addr in (0x000000, 0xFFFFFF):
        _ctr[0] += 1
        addr = _ADDR_BLOCK_BASE + (_ctr[0] % _ADDR_BLOCK_SIZE)
    return addr, f"SIM{_ctr[0]:03d}", _ctr[0]


class SimAircraft:
    __slots__ = ("callsign", "track_no",
                 "waypoints", "alt_ft", "alt_target_ft", "vrate_fpm",
                 "climb_fpm", "speed_kt",
                 "loop", "_seg", "_seg_t", "_lat", "_lon", "_hdg",
                 "turn_rate_deg_s",
                 "_adsb_due", "_pseudo_due", "spi_until",
                 "mode1", "mode2", "mode3a", "modes_addr",
                 "xpdr1", "xpdr2", "xpdr3a", "xpdrC", "xpdrS",
                 "_lockout_until", "_last_sel_scan")

    def __init__(self, alt_ft=35000, speed_kt=450, mode2=None):
        self.modes_addr, self.callsign, self.track_no = _new_id()
        self.waypoints = []
        self.alt_ft        = float(alt_ft)
        self.alt_target_ft = float(alt_ft)
        self.vrate_fpm     = 0.0
        self.climb_fpm     = _CLIMB_FPM
        self.speed_kt  = speed_kt
        self.loop      = False
        self._seg      = 0
        self._seg_t    = 0.0
        self._lat      = None
        self._lon      = None
        self._hdg      = None        # reported heading; None until a course exists
        # 0 means turn instantly.  This is a test tool: being able to command
        # an impossible heading step is a feature, not a defect, so the
        # rate limit is opt-in rather than imposed.
        self.turn_rate_deg_s = _DEFAULT_TURN_RATE
        # Each message type carries its own next-due time (5.3).
        self._adsb_due = {}
        self._pseudo_due = {}
        self.spi_until = 0.0
        # Mode 1 is stored in display form: A digit 0-7, B digit 0-3.
        self.mode1      = random.randint(0, 7) * 8 + random.randint(0, 3)
        # Mode 2 is a unit code, so it is inherited from the radar's default
        # rather than randomised per aircraft (6.4).
        self.mode2      = random.randint(0, 0o7777) if mode2 is None else mode2
        self.mode3a     = random.randint(0, 0o7777)
        self.xpdr1 = self.xpdr2 = self.xpdr3a = self.xpdrC = self.xpdrS = True
        self._lockout_until = 0.0
        self._last_sel_scan = -1

    _MODE_FLAG = {
        iff.MODE_1:     "xpdr1",
        iff.MODE_2:     "xpdr2",
        iff.MODE_3A:    "xpdr3a",
        iff.MODE_C:     "xpdrC",
        iff.MODE_S_AC:  "xpdrS",
        iff.MODE_S_SEL: "xpdrS",
    }

    def has_xpdr(self, mode_code):
        attr = self._MODE_FLAG.get(mode_code)
        if attr is None:
            raise ValueError(f"unknown mode {mode_code!r}")
        return getattr(self, attr)

    @property
    def icao(self):
        return f"{self.modes_addr:06X}"

    @property
    def lat(self):
        return self._lat if self._lat is not None else (
            self.waypoints[0][0] if self.waypoints else None)

    @property
    def lon(self):
        return self._lon if self._lon is not None else (
            self.waypoints[0][1] if self.waypoints else None)

    def _n_segs(self):
        n = len(self.waypoints)
        if n < 2:
            return 0
        return n if self.loop else n - 1

    def _seg_ends(self, i):
        wps = self.waypoints
        return wps[i], wps[(i + 1) % len(wps)]

    def course(self):
        """The raw bearing of the segment being flown — steps discontinuously
        at each waypoint, so it is not what gets reported."""
        nseg = self._n_segs()
        if nseg == 0:
            return 0.0
        a, b = self._seg_ends(min(self._seg, nseg - 1))
        return iff.bearing_range_nm(a[0], a[1], b[0], b[1])[0]

    def heading(self):
        """The reported heading.  Equals course() when turn_rate_deg_s is 0,
        otherwise lags it by at most that many degrees per second."""
        return self.course() if self._hdg is None else self._hdg

    def _step_heading(self, dt):
        # With fewer than two waypoints there is no course to hold.  Latching
        # course()'s 0.0 placeholder here would peg the aircraft to north and
        # then walk it to the real course at the turn rate — which is what
        # happens to every aircraft created by clicking its first waypoint.
        if self._n_segs() == 0:
            return
        want = self.course()
        rate = self.turn_rate_deg_s
        if rate <= 0.0 or self._hdg is None:
            self._hdg = want            # instant: report the course as flown
            return
        err = iff.angle_diff(want, self._hdg)
        limit = rate * dt
        if abs(err) <= limit:
            self._hdg = want
        else:
            self._hdg = (self._hdg + limit * (1.0 if err > 0 else -1.0)) % 360.0

    def _step_altitude(self, dt):
        """Fly toward the target altitude at the climb rate, and report the
        vertical rate actually being achieved."""
        err = self.alt_target_ft - self.alt_ft
        if abs(err) < 1e-6:
            self.vrate_fpm = 0.0
            return
        step = self.climb_fpm * dt / 60.0
        if abs(err) <= step:
            self.alt_ft = self.alt_target_ft
            self.vrate_fpm = 0.0
        else:
            self.alt_ft += step * (1.0 if err > 0 else -1.0)
            self.vrate_fpm = self.climb_fpm * (1.0 if err > 0 else -1.0)

    def step(self, dt):
        self._step_altitude(dt)
        self._step_heading(dt)
        wps = self.waypoints
        nseg = self._n_segs()
        if nseg == 0 or self.speed_kt <= 0:
            if self._lat is None and wps:
                self._lat, self._lon = wps[0]
            return

        if self._seg >= nseg:
            if self.loop:
                self._seg = 0
            else:
                self._lat, self._lon = wps[-1]
                return

        self._seg_t += dt
        for _ in range(nseg + 2):
            a, b = self._seg_ends(self._seg)
            stime = (iff.bearing_range_nm(a[0], a[1], b[0], b[1])[1] / self.speed_kt * 3600.0
                     if self.speed_kt > 0 else 1e9)
            if stime <= 0.0:
                self._seg += 1
            elif self._seg_t < stime:
                t = self._seg_t / stime
                self._lat = a[0] + t * (b[0] - a[0])
                self._lon = a[1] + t * (b[1] - a[1])
                return
            else:
                self._seg_t -= stime
                self._seg += 1
            if self._seg >= nseg:
                if self.loop:
                    self._seg = 0
                else:
                    self._lat, self._lon = wps[-1]
                    self._seg_t = 0.0
                    return
        self._lat, self._lon = wps[0]
        self._seg_t = 0.0

    # ── Emitter ───────────────────────────────────────────────────────────────
    #
    # These two methods are the ONLY way aircraft state reaches the display.
    # Nothing outside this class may read .lat/.lon/.mode3a/.alt_ft on the
    # receive path — the exceptions are step() above (physics) and channel.py
    # (which needs true position for range and line-of-sight).  Do not add a
    # getter here that leaks a field straight to the display: the whole point
    # of the split is that the picture is built by decoding frames.

    def iff_reply(self, mode, prt_no, qnh_hpa=iff.STD_QNH_HPA):
        """Encode this aircraft's reply to one interrogation, or None if it has
        no transponder for that mode / no position yet."""
        if self._lat is None or self._lon is None:
            return None
        if not self.has_xpdr(mode):
            return None

        kw = {}
        if mode == iff.MODE_1:
            kw["mission_code"] = self.mode1
        elif mode == iff.MODE_2:
            kw["unit_code"] = self.mode2
        elif mode == iff.MODE_3A:
            kw["squawk"] = self.mode3a
        elif mode == iff.MODE_C:
            # Mode C reports PRESSURE altitude in 100 ft steps, not the
            # aircraft's geometric height at 1 ft resolution.
            kw["alt_ft"] = iff.encode_mode_c_alt(self.alt_ft, qnh_hpa)
        elif mode == iff.MODE_S_AC:
            kw["modes_addr"] = self.modes_addr
        elif mode == iff.MODE_S_SEL:
            kw["modes_addr"] = self.modes_addr
            kw["bds_reg"]    = iff.BDS_CALLSIGN
            kw["callsign"]   = self.callsign
        else:
            raise ValueError(f"unknown mode {mode!r}")

        return iff.encode_target_reply(self.modes_addr, mode, prt_no,
                                       self._lat, self._lon, **kw)

    def pseudo_frame(self, now, fmt):
        """Emit the next due custom-format frame as (msg_name, hex), or None.

        Same scheduling shape as adsb_frame, but the periods come from the
        config rather than from DO-260B, which is the "custom time periods"
        part of the requirement.
        """
        if self._lat is None or self._lon is None:
            return None

        due = None
        for name, msg in fmt.messages.items():
            t = self._pseudo_due.get(name)
            if t is None:
                # Stagger first emissions so all types are not due at once.
                self._pseudo_due[name] = now + random.uniform(0.0, msg.period)
                continue
            if now >= t and (due is None or t < self._pseudo_due[due]):
                due = name
        if due is None:
            return None

        msg = fmt.messages[due]
        lo = max(1e-3, msg.period - msg.jitter)
        self._pseudo_due[due] = now + random.uniform(lo, msg.period + msg.jitter)
        return due, fmt.encode(due, self)

    def adsb_frame(self, now, qnh_hpa=iff.STD_QNH_HPA):
        """Emit the next due ADS-B squitter as (label, hex_frame), or None.

        Each message type runs on its own scheduler at DO-260B nominal rates
        (5.3): airborne position and airborne velocity about 2 Hz each with a
        uniformly random interval, aircraft identification about every 5 s.
        The old fixed 6-slot cycle emitted 12.5 msg/s per aircraft — position
        at 8.3 Hz and ident at 2 Hz, both far too fast.

        Frames are hex strings rather than bytes because that is the wire
        framing dump1090 emits and the whole decode path already speaks it.
        """
        if self._lat is None or self._lon is None:
            return None

        due = None
        for kind in ("POS", "VEL", "IDENT"):
            t = self._adsb_due.get(kind)
            if t is None:
                # Stagger first emissions so all types are not due at once.
                lo, hi = _ADSB_INTERVAL[kind]
                self._adsb_due[kind] = now + random.uniform(0.0, hi)
                continue
            if now >= t and (due is None or t < self._adsb_due[due]):
                due = kind
        if due is None:
            return None

        lo, hi = _ADSB_INTERVAL[due]
        self._adsb_due[due] = now + random.uniform(lo, hi)
        icao = self.icao

        if due == "IDENT":
            return "IDENT", build_identification(icao, self.callsign)
        if due == "VEL":
            # No path, or stopped, means no ground track to report.  Emitting a
            # velocity frame anyway would claim a fabricated heading — and
            # since course() falls back to 0.0, that heading is due north, so
            # every aircraft would be "detected" northbound and then swing
            # round to its real track at the turn rate.  Report nothing: the
            # receiver keeps no track angle and the plot stays a circle, which
            # is the honest picture.
            if self._n_segs() == 0 or self.speed_kt <= 0:
                return None
            return "VEL", build_velocity(icao, self.speed_kt, self.heading(),
                                         int(round(self.vrate_fpm)))
        # Position alternates even/odd CPR frames.
        self._adsb_due["_odd"] = odd = not self._adsb_due.get("_odd", False)
        alt = iff.encode_mode_c_alt(self.alt_ft, qnh_hpa)
        return ("POS-O" if odd else "POS-E",
                build_position(icao, self._lat, self._lon, alt, odd))


# ── App ───────────────────────────────────────────────────────────────────────

class CombinedApp(tk.Tk):

    def __init__(self, c_lat, c_lon, rng, declination=0.0, cfg_path=None):
        super().__init__()
        self.title("IFF Radar")
        self.configure(bg=ui.PANEL)
        self.resizable(True, True)
        self.minsize(ui.CANVAS_SZ + _PANEL_W + _REPLY_W + round(20 * ui.SCALE),
                     round(560 * ui.SCALE))

        self.c_lat, self.c_lon, self.rng = c_lat, c_lon, rng
        self.declination = declination
        self._tick = time.monotonic()
        self._cw = self._ch = ui.CANVAS_SZ
        self._fullscreen = False
        self._cursor = None
        self._cursor_on = True
        self._truth_on  = False

        self._aircraft: list[SimAircraft] = []
        self._selected: SimAircraft | None = None
        self._drag_wp   = None
        self._draw_from = None
        self._dirty     = True
        self._bg_sig    = None
        self._routes_dirty = True
        self._fg_sig    = None
        self._beam_sig  = None
        # 6.5 — cached canvas item ids per track, so the colour fade is an
        # itemconfig rather than a delete + recreate every 0.125 s.
        self._blip_items: dict[int, list] = {}
        self._blip_fade: dict[int, int] = {}

        # Sweep state
        self._sweep_anchor_t  = time.monotonic()
        self._sweep_anchor_az = 0.0
        self._sweep_rpm       = _DEFAULT_RPM

        # PRT bookkeeping
        self._prt_no = 0
        self._interro_accum = 0.0
        self._scan_no = 0
        self._garble_count = 0
        self._unit2 = _DEFAULT_UNIT_CODE
        self._dupe_addrs: set[int] = set()

        # Emitter -> channel -> receiver.  The receiver owns the track store;
        # the app only reads it to draw.  Colours are assigned by address so a
        # track keeps its colour across a decay-and-reacquire cycle.
        # Custom 1090 format.  A bad config must not stop the simulator from
        # starting — it reports the problem and falls back to standard ADS-B.
        self._cfg_path = cfg_path
        self.fmt = None
        self.fmt_error = None
        try:
            self.fmt = pseudo1090.load(cfg_path)
        except pseudo1090.ConfigError as e:
            self.fmt_error = str(e)
        self._tx_mode = (self.fmt.mode if self.fmt
                         else pseudo1090.MODE_STANDARD)
        # Append-only 1090 airtime log, newest last, capped.
        self._log: list[tuple] = []
        self._log_dirty = True

        self.site = channel.RadarSite(lat=c_lat, lon=c_lon)
        self._colors: dict[int, str] = {}
        self.rx = Receiver(self.site, colour_fn=self._color_for)

        self._target_menu_map: dict[str, int] = {}
        # Content signature of the dropdown; the menu is rebuilt only when
        # this changes, so receiving frames cannot churn it (see
        # _refresh_target_menu).
        self._target_menu_sig: tuple | None = None

        self._selected_addr: int | None = None
        self._table_dirty = False
        self._table_last  = 0.0
        self._row_addrs: list[int] = []

        # Tk vars — interrogation panel
        self._v_mode   = tk.StringVar(value=_MODE_LABELS[2][0])
        self._v_target = tk.StringVar(value="(no known address)")
        self._v_rpm    = tk.IntVar(value=_DEFAULT_RPM)
        self._v_bw     = tk.DoubleVar(value=_DEFAULT_BW)
        self._v_prt    = tk.IntVar(value=_DEFAULT_PRT)
        self._v_qnh    = tk.DoubleVar(value=iff.STD_QNH_HPA)
        self._v_unit2  = tk.StringVar(value=f"{_DEFAULT_UNIT_CODE:04o}")

        # Snapshots of Tk vars for the main loop
        self._rpm_snap        = _DEFAULT_RPM
        self._bw_snap         = float(_DEFAULT_BW)
        self._prt_s_snap      = _DEFAULT_PRT * 1e-6
        self._qnh_snap        = iff.STD_QNH_HPA
        self._mode_label_snap = _MODE_LABELS[2][0]
        self._target_addr_snap: int | None = None

        # Tk vars — aircraft editor
        self._v_name = tk.StringVar(value="—")
        self._v_icao = tk.StringVar()
        self._v_call = tk.StringVar()
        self._v_alt  = tk.IntVar(value=35000)
        self._v_spd  = tk.IntVar(value=450)
        self._v_turn = tk.DoubleVar(value=_DEFAULT_TURN_RATE)
        self._v_loop = tk.BooleanVar(value=False)
        self._v_m1   = tk.StringVar()
        self._v_m2   = tk.StringVar()
        self._v_m3a  = tk.StringVar()
        self._v_x1  = tk.BooleanVar(value=True)
        self._v_x2  = tk.BooleanVar(value=True)
        self._v_x3a = tk.BooleanVar(value=True)
        self._v_xc  = tk.BooleanVar(value=True)
        self._v_xs  = tk.BooleanVar(value=True)
        self._v_cur    = tk.StringVar()
        self._v_dwell  = tk.StringVar(value="hits/dwell —")
        self._v_status = tk.StringVar(value="")
        self._v_truth  = tk.BooleanVar(value=False)
        self._v_txmode = tk.StringVar(value=self._tx_mode)
        self._v_logstat = tk.StringVar(value="")
        self._v_fmtinfo = tk.StringVar(value="")

        self._build_ui()
        self._refresh_fmtinfo()
        self.bind("<F11>",    self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._loop()

    def _on_close(self):
        self.destroy()

    # ── 1090 MHz airtime log ──────────────────────────────────────────────────

    def _log_1090(self, kind, label, frame, decoded=True, lost=False):
        """Record one transmission.  Append-only and capped, unlike every other
        text view here, which is a snapshot rebuilt each refresh."""
        self._log.append((time.monotonic(), kind, label, frame, decoded, lost))
        if len(self._log) > _LOG_MAX:
            del self._log[:len(self._log) - _LOG_MAX]
        self._log_dirty = True

    def _apply_txmode(self, _val=None):
        self._tx_mode = self._v_txmode.get()
        if self._tx_mode != pseudo1090.MODE_STANDARD and self.fmt is None:
            # Nothing to transmit: say so instead of going silently dead.
            self._v_txmode.set(pseudo1090.MODE_STANDARD)
            self._tx_mode = pseudo1090.MODE_STANDARD
        self._refresh_fmtinfo()

    def _reload_fmt(self):
        """Re-read the config without restarting, so editing the cfg is a fast
        loop.  A bad file leaves the previous format in place."""
        try:
            self.fmt = pseudo1090.load(self._cfg_path)
            self.fmt_error = None
            for ac in self._aircraft:
                ac._pseudo_due.clear()
        except pseudo1090.ConfigError as e:
            self.fmt_error = str(e)
        self._refresh_fmtinfo()

    def _refresh_fmtinfo(self):
        if self.fmt_error:
            first = self.fmt_error.strip().splitlines()
            n = max(0, len(first) - 1)
            self._v_fmtinfo.set(f"config error ({n} problem"
                                f"{'' if n == 1 else 's'}) — see terminal; "
                                f"transmitting standard ADS-B")
            self._fmt_lbl.config(fg=_DWELL_LOW_COL)
            print(f"[pseudo1090] {self.fmt_error}")
            return
        if self.fmt is None:
            self._v_fmtinfo.set("no custom format loaded")
            self._fmt_lbl.config(fg=ui.FG_DIM)
            return
        names = ", ".join(self.fmt.messages)
        bits = "addr in every msg" if self.fmt.addr_field else (
            "addr in one msg" if self.fmt.has_address else "NO ADDRESS")
        self._v_fmtinfo.set(f"{os.path.basename(self.fmt.path)}: "
                            f"{len(self.fmt.messages)} msgs ({names}); {bits}")
        self._fmt_lbl.config(
            fg=_DWELL_LOW_COL if not self.fmt.has_address else ui.FG_DIM)

    # ── 1090 airtime log ──────────────────────────────────────────────────────

    _LOG_FMT = "{t:>7} {kind:<6} {label:<9} {st:<4} {hex}"

    def _flush_log(self):
        if not self._log_dirty:
            return
        self._log_dirty = False
        t0 = self._log[0][0] if self._log else 0.0
        w = self._log_txt
        # Only autoscroll when already at the bottom, so scrolling back to read
        # something is not yanked away on the next frame.
        at_end = w.yview()[1] >= 0.999
        w.config(state=tk.NORMAL)
        w.delete("1.0", tk.END)
        for ts, kind, label, frame, decoded, lost in self._log:
            if lost:
                st, tag = "lost", "lost"
            elif not decoded:
                st, tag = "??", "nodec"
            else:
                st, tag = "ok", ("custom" if kind == "CUSTOM" else "ok")
            w.insert(tk.END, self._LOG_FMT.format(
                t=f"{ts - t0:6.1f}", kind=kind, label=(label or "")[:9],
                st=st, hex=frame or "") + "\n", tag)
        w.config(state=tk.DISABLED)
        if at_end:
            w.see(tk.END)
        rx = self.rx
        self._v_logstat.set(f"{len(self._log)} tx  "
                            f"{rx.undecodable} undec  "
                            f"{rx.unattributed} unattr")

    # ── Track store (owned by the receiver) ───────────────────────────────────

    @property
    def _tracks(self):
        return self.rx.tracks

    @property
    def _known_addrs(self):
        return self.rx.known_addrs

    def _color_for(self, addr):
        col = self._colors.get(addr)
        if col is None:
            col = self._colors[addr] = ui.random_color()
        return col

    # ── Panel UI ──────────────────────────────────────────────────────────────

    def _button(self, parent, text, cmd, bg=ui.BTN, fg=ui.FG, active=None):
        b = ui.flat_button(parent, text, cmd, bg=bg, fg=fg, active=active)
        b.pack(fill=tk.X, padx=ui.PAD, pady=round(2 * ui.SCALE))
        return b

    def _build_ui(self):
        # ── Left panel: LAST REPLY ────────────────────────────────────────────
        lp = ui.make_panel(self, side=tk.LEFT, width=_REPLY_W)
        self._left_panel = lp

        tk.Frame(lp, bg=ui.PANEL, height=round(10 * ui.SCALE)).pack()
        tk.Label(lp, text="LAST REPLY", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(lp)

        self._detail = tk.Text(lp, bg=ui.ENTRY, fg="#888888",
                               font=ui.F_SM, relief=tk.FLAT, bd=0,
                               wrap=tk.WORD, highlightthickness=0,
                               state=tk.DISABLED,
                               cursor="arrow")
        self._detail.tag_configure("hdr", foreground=ui.FG)
        self._detail.tag_configure("row", foreground="#aaaaaa")
        self._detail.tag_configure("stale", foreground="#555555")
        self._detail_scroll = tk.Scrollbar(lp, command=self._detail.yview,
                                           bg=ui.PANEL, troughcolor=ui.PANEL,
                                           activebackground=ui.PANEL)
        self._detail.config(yscrollcommand=self._detail_scroll.set)
        self._detail.pack(side=tk.TOP, fill=tk.BOTH, expand=True,
                          padx=ui.PAD, pady=ui.PAD2)
        self._detail_scroll.place(in_=self._detail, relx=1.0, rely=0,
                                  relheight=1.0, anchor="ne")
        self._detail.config(state=tk.NORMAL)
        self._detail.insert(tk.END,
                            "IFF  (click a track)\n\n"
                            "ADS-B  (click a track)")
        self._detail.config(state=tk.DISABLED)

        # ── Left panel, lower half: 1090 MHz airtime ──────────────────────────
        # The only append-only view in the app.  Everything else here is a
        # snapshot rebuilt each refresh; this is a running record of what was
        # actually transmitted and whether the receiver made sense of it.
        ui.sep(lp)
        hdr = tk.Frame(lp, bg=ui.PANEL)
        hdr.pack(fill=tk.X, padx=ui.PAD)
        tk.Label(hdr, text="1090 AIRTIME", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(side=tk.LEFT)
        tk.Label(hdr, textvariable=self._v_logstat, bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_SM, anchor="e").pack(side=tk.RIGHT)
        ui.sep(lp)

        self._log_txt = tk.Text(lp, bg=ui.ENTRY, fg="#888888", font=ui.F_SM,
                                relief=tk.FLAT, bd=0, wrap=tk.NONE,
                                highlightthickness=0, state=tk.DISABLED,
                                cursor="arrow", height=10)
        self._log_txt.tag_configure("ok", foreground="#aaaaaa")
        self._log_txt.tag_configure("custom", foreground="#7fd0c0")
        self._log_txt.tag_configure("lost", foreground=_LOG_LOST_COL)
        self._log_txt.tag_configure("nodec", foreground=_DWELL_LOW_COL)
        self._log_scroll = tk.Scrollbar(lp, command=self._log_txt.yview,
                                        bg=ui.PANEL, troughcolor=ui.PANEL,
                                        activebackground=ui.PANEL)
        self._log_txt.config(yscrollcommand=self._log_scroll.set)
        self._log_txt.pack(side=tk.TOP, fill=tk.BOTH, expand=True,
                           padx=ui.PAD, pady=ui.PAD2)
        self._log_scroll.place(in_=self._log_txt, relx=1.0, rely=0,
                               relheight=1.0, anchor="ne")

        # ── Canvas ────────────────────────────────────────────────────────────
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

        # Scrollable: this panel carries more controls than fit in a short
        # window, and silently clipping half of them is worse than a bar.
        p = ui.make_scroll_panel(self, side=tk.RIGHT, width=_PANEL_W)
        self._panel = p

        # ── 1090 FORMAT ──
        tk.Frame(p, bg=ui.PANEL, height=round(10 * ui.SCALE)).pack()
        tk.Label(p, text="1090 FORMAT", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)

        tmf = tk.Frame(p, bg=ui.PANEL)
        tmf.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        tk.Label(tmf, text="transmit", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        tm = tk.OptionMenu(tmf, self._v_txmode, *pseudo1090.MODES,
                           command=self._apply_txmode)
        tm.config(bg=ui.ENTRY, fg=ui.FG, activebackground=ui.BTN_ACT,
                  font=ui.F_MD, relief=tk.FLAT, bd=0, highlightthickness=0)
        tm["menu"].config(bg=ui.ENTRY, fg=ui.FG, font=ui.F_MD)
        tm.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self._fmt_lbl = tk.Label(p, textvariable=self._v_fmtinfo, bg=ui.PANEL,
                                 fg=ui.FG_DIM, font=ui.F_SM, anchor="w",
                                 justify=tk.LEFT, wraplength=_PANEL_W - 4 * ui.PAD)
        self._fmt_lbl.pack(fill=tk.X, padx=ui.PAD, pady=(0, ui.PAD2))
        self._button(p, "Reload format cfg", self._reload_fmt)

        # ── INTERROGATION ──
        ui.sep(p)
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
        ui.slider_row(p, "qnh hPa", self._v_qnh, 950.0, 1050.0, resolution=0.25)
        # Mode 2 is a UNIT code — one value per unit, inherited by new
        # aircraft, still editable per aircraft afterwards.
        e_u2 = ui.entry_row(p, "unit M2", self._v_unit2)
        e_u2.bind("<Return>",   self._apply_unit2)
        e_u2.bind("<FocusOut>", self._apply_unit2)

        df = tk.Frame(p, bg=ui.PANEL)
        df.pack(fill=tk.X, padx=ui.PAD, pady=(0, ui.PAD2))
        self._dwell_lbl = tk.Label(df, textvariable=self._v_dwell, bg=ui.PANEL,
                                   fg=ui.FG_DIM, font=ui.F_SM, anchor="w")
        self._dwell_lbl.pack(side=tk.LEFT)
        # Interrogation status: warns when Mode S Selective is armed with no
        # target, which would otherwise fall back to addr 0 and go silent.
        self._status_lbl = tk.Label(p, textvariable=self._v_status, bg=ui.PANEL,
                                    fg=ui.FG_DIM, font=ui.F_SM, anchor="w")
        self._status_lbl.pack(fill=tk.X, padx=ui.PAD, pady=(0, ui.PAD2))

        # ── AIRCRAFT ──
        ui.sep(p)
        tk.Label(p, text="AIRCRAFT", bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)
        ui.sep(p)
        self._button(p, "+ New", self._new_ac)

        self._lb = tk.Listbox(p, bg=ui.ENTRY, fg=ui.FG,
                              selectbackground="#222222",
                              selectforeground="#ffffff", font=ui.F_MD,
                              relief=tk.FLAT, bd=0, height=4,
                              activestyle="none", highlightthickness=0)
        self._lb.pack(fill=tk.X, padx=ui.PAD, pady=(ui.PAD2, 0))
        self._lb.bind("<<ListboxSelect>>", self._lb_sel)

        ui.sep(p)
        tk.Label(p, textvariable=self._v_name, bg=ui.PANEL, fg=ui.FG,
                 font=ui.F_BLD, anchor="w").pack(fill=tk.X, padx=ui.PAD)

        e_icao = ui.entry_row(p, "address",  self._v_icao)
        e_call = ui.entry_row(p, "callsign", self._v_call)
        for e in (e_icao, e_call):
            e.bind("<Return>",   self._apply_id)
            e.bind("<FocusOut>", self._apply_id)

        ui.slider_row(p, "alt ft",   self._v_alt, -1000, 50000, 25,
                      command=self._apply_sel)
        ui.slider_row(p, "speed kt", self._v_spd,     0,  4088,  1,
                      command=self._apply_sel)
        ui.slider_row(p, "turn °/s", self._v_turn, 0.0, _TURN_RATE_MAX, 0.5,
                      command=self._apply_sel)

        lf = tk.Frame(p, bg=ui.PANEL)
        lf.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        tk.Label(lf, text="loop", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        tk.Checkbutton(lf, variable=self._v_loop, bg=ui.PANEL,
                       fg=ui.FG, selectcolor=ui.ENTRY,
                       activebackground=ui.PANEL,
                       command=self._toggle_loop).pack(side=tk.LEFT)

        # ── IFF ──
        ui.sep(p)
        tk.Label(p, text="IFF", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, anchor="w").pack(fill=tk.X, padx=ui.PAD)

        e_m1  = ui.entry_row(p, "M1 code",  self._v_m1)
        e_m2  = ui.entry_row(p, "M2 code",  self._v_m2)
        e_m3a = ui.entry_row(p, "M3A sqwk", self._v_m3a)
        # The Mode S address IS the ICAO address — one variable, two views, so
        # they cannot drift apart.  Editing either one edits the aircraft.
        e_msa = ui.entry_row(p, "MS addr",  self._v_icao)
        e_msa.bind("<Return>",   self._apply_id)
        e_msa.bind("<FocusOut>", self._apply_id)
        for e in (e_m1, e_m2, e_m3a):
            e.bind("<Return>",   self._apply_iff)
            e.bind("<FocusOut>", self._apply_iff)

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

        tfr = tk.Frame(p, bg=ui.PANEL)
        tfr.pack(fill=tk.X, padx=ui.PAD, pady=ui.PAD2)
        tk.Label(tfr, text="truth", bg=ui.PANEL, fg=ui.FG_DIM,
                 font=ui.F_MD, width=9, anchor="w").pack(side=tk.LEFT)
        tk.Checkbutton(tfr, variable=self._v_truth, bg=ui.PANEL,
                       fg=ui.FG, selectcolor=ui.ENTRY,
                       activebackground=ui.PANEL, font=ui.F_SM,
                       text="overlay ground truth",
                       command=self._toggle_truth).pack(side=tk.LEFT)

        self._button(p, "IDENT (SPI 18s)", self._ident)
        self._button(p, "Delete track", self._del_ac, fg="#888888")
        self._button(p, "Reset positions", self._reset_positions,
                     bg=ui.BTN_RED, fg="#ffffff", active=ui.BTN_RED_A)

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
                            state=tk.DISABLED, height=8)
        self._tbl.pack(fill=tk.BOTH, expand=True)
        self._tbl.bind("<Button-1>", self._on_table_click)
        self._tbl.tag_configure("row", foreground=ui.FG)
        self._tbl.tag_configure("sel", background="#2a2a2a", foreground="#ffffff")
        self._tbl.tag_configure("emerg", foreground=_EMERG_COL)
        self._tbl.tag_configure("spi", foreground="#66c0ff")

        # Footer
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
        tk.Label(p, textvariable=self._v_cur, bg=ui.PANEL, fg="#444444",
                 font=ui.F_SM, anchor="w"
                 ).pack(fill=tk.X, padx=ui.PAD, pady=(0, round(6 * ui.SCALE)))

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def _to_xy(self, lat, lon):
        cx, cy, r = ui.geom(self._cw, self._ch)
        return ui.ll_to_xy(lat, lon, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    def _to_ll(self, x, y):
        cx, cy, r = ui.geom(self._cw, self._ch)
        return ui.xy_to_ll(x, y, cx, cy, r, self.c_lat, self.c_lon, self.rng)

    def _mag(self, true_deg):
        return (true_deg - self.declination) % 360.0

    def _nearest_wp(self, x, y, only=None):
        best, bd = None, ui.HIT_WP * ui.scale_for(self._cw, self._ch)
        acs = [only] if only is not None else self._aircraft
        for ac in acs:
            for i, (la, lo) in enumerate(ac.waypoints):
                pt = self._to_xy(la, lo)
                if pt and math.hypot(x - pt[0], y - pt[1]) < bd:
                    best, bd = (ac, i), math.hypot(x - pt[0], y - pt[1])
        return best

    def _nearest_ac(self, x, y):
        best, best_d = None, ui.HIT_WP * ui.scale_for(self._cw, self._ch) * 4.0
        for ac in self._aircraft:
            if ac.lat is None or ac.lon is None:
                continue
            pt = self._to_xy(ac.lat, ac.lon)
            if pt is None:
                continue
            d = math.hypot(x - pt[0], y - pt[1])
            if d < best_d:
                best_d = d
                best = ac
        return best

    def _on_resize(self, ev):
        self._cw, self._ch = ev.width, ev.height

    # ── Canvas interaction ────────────────────────────────────────────────────

    def _press(self, ev):
        if self._selected is not None:
            hit = self._nearest_wp(ev.x, ev.y, only=self._selected)
            if hit:
                self._drag_wp = hit
                return
        hit = self._nearest_wp(ev.x, ev.y)
        if hit:
            self._select(hit[0])
            return
        ac = self._nearest_ac(ev.x, ev.y)
        if ac is not None:
            self._select(ac)
            return
        ll = self._to_ll(ev.x, ev.y)
        if ll:
            if self._selected is None:
                self._new_ac()
            self._selected.waypoints.append(ll)
        self._draw_from = (ev.x, ev.y)
        self._dirty = self._routes_dirty = True

    def _drag(self, ev):
        if self._drag_wp:
            ll = self._to_ll(ev.x, ev.y)
            if ll:
                ac, i = self._drag_wp
                if i < len(ac.waypoints):
                    ac.waypoints[i] = ll
                self._routes_dirty = True
            return
        if self._draw_from is not None and self._selected is not None:
            lx, ly = self._draw_from
            dist = math.hypot(ev.x - lx, ev.y - ly)
            spacing = max(1, round(_DRAW_SPACING_PX * ui.scale_for(self._cw, self._ch)))
            if dist >= spacing:
                ux, uy = (ev.x - lx) / dist, (ev.y - ly) / dist
                pts = []
                px, py = lx, ly
                for _ in range(int(dist // spacing)):
                    px += ux * spacing
                    py += uy * spacing
                    ll = self._to_ll(px, py)
                    if ll:
                        pts.append(ll)
                if pts:
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

    def _rclick(self, ev):
        hit = self._nearest_wp(ev.x, ev.y)
        if hit:
            ac, i = hit
            if i < len(ac.waypoints):
                ac.waypoints.pop(i)
            self._dirty = self._routes_dirty = True
        else:
            self._cursor_on = not self._cursor_on

    # ── Aircraft management ───────────────────────────────────────────────────

    def _new_ac(self):
        ac = SimAircraft(mode2=self._unit2)
        self._aircraft.append(ac)
        self._select(ac)
        self._dirty = True

    def _select(self, ac):
        self._selected = ac
        self._routes_dirty = True
        self._v_name.set(f"{ac.icao}  {ac.callsign}")
        self._v_icao.set(ac.icao)
        self._v_call.set(ac.callsign)
        self._v_alt.set(int(round(ac.alt_target_ft)))
        self._v_spd.set(int(ac.speed_kt))
        self._v_turn.set(ac.turn_rate_deg_s)
        self._v_loop.set(ac.loop)
        self._v_m1.set(f"{ac.mode1:02o}")
        self._v_m2.set(f"{ac.mode2:04o}")
        self._v_m3a.set(f"{ac.mode3a:04o}")
        self._v_x1.set(ac.xpdr1)
        self._v_x2.set(ac.xpdr2)
        self._v_x3a.set(ac.xpdr3a)
        self._v_xc.set(ac.xpdrC)
        self._v_xs.set(ac.xpdrS)
        idx = next((i for i, a in enumerate(self._aircraft) if a is ac), None)
        if idx is not None:
            self._lb.selection_clear(0, tk.END)
            self._lb.selection_set(idx)
        self._selected_addr = ac.modes_addr
        self._table_dirty = True

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

    def _toggle_truth(self):
        self._truth_on = self._v_truth.get()

    def _toggle_loop(self):
        if self._selected:
            self._selected.loop = self._v_loop.get()
            self._routes_dirty = True

    def _apply_sel(self, _val=None):
        if not self._selected:
            return
        self._selected.alt_target_ft = float(self._v_alt.get())
        self._selected.turn_rate_deg_s = float(self._v_turn.get())
        self._selected.speed_kt = self._v_spd.get()

    def _apply_id(self, _ev=None):
        ac = self._selected
        if not ac:
            return
        raw = self._v_icao.get().strip().upper()
        call = self._v_call.get().strip().upper()
        try:
            addr = int(raw, 16) & 0xFFFFFF
            if addr not in (0x000000, 0xFFFFFF) and len(raw) == 6:
                for other in self._aircraft:
                    if other is not ac and other.modes_addr == addr:
                        self._v_icao.set(ac.icao)
                        self._v_name.set(f"{ac.icao}  {ac.callsign}")
                        return
                ac.modes_addr = addr
        except ValueError:
            pass
        if call:
            ac.callsign = call[:8]
        self._v_icao.set(ac.icao)
        self._v_call.set(ac.callsign)
        self._v_name.set(f"{ac.icao}  {ac.callsign}")
        self._selected_addr = ac.modes_addr
        self._dirty = True
        self._table_dirty = True

    def _apply_iff(self, _ev=None):
        ac = self._selected
        if not ac:
            return
        def parse_oct(s, fallback, max_val):
            try:
                v = int(s.strip(), 8)
                return v if 0 <= v <= max_val else fallback
            except ValueError:
                return fallback
        # Mode 1 is A(3 bits) + B(2 bits): first octal digit 0-7, second 0-3,
        # i.e. 00-73 octal, 32 valid codes.  Reject rather than silently fold.
        m1 = parse_oct(self._v_m1.get(), ac.mode1, iff.MODE_1_MAX_DISPLAY)
        ac.mode1 = m1 if iff.valid_mode1(m1) else ac.mode1
        ac.mode2      = parse_oct(self._v_m2.get(),  ac.mode2,  0o7777)
        ac.mode3a     = parse_oct(self._v_m3a.get(), ac.mode3a, 0o7777)
        self._v_m1.set(f"{ac.mode1:02o}")
        self._v_m2.set(f"{ac.mode2:04o}")
        self._v_m3a.set(f"{ac.mode3a:04o}")

    def _apply_unit2(self, _ev=None):
        """Mode 2 is a unit code: set the radar-level default that new aircraft
        inherit.  Existing aircraft keep whatever they were given."""
        try:
            v = int(self._v_unit2.get().strip(), 8)
            if 0 <= v <= 0o7777:
                self._unit2 = v
        except ValueError:
            pass
        self._v_unit2.set(f"{self._unit2:04o}")

    def _ident(self):
        """SPI / ident pulse on the selected aircraft."""
        if self._selected:
            self._selected.spi_until = time.monotonic() + _SPI_S

    def _apply_caps(self):
        ac = self._selected
        if not ac:
            return
        ac.xpdr1  = self._v_x1.get()
        ac.xpdr2  = self._v_x2.get()
        ac.xpdr3a = self._v_x3a.get()
        ac.xpdrC  = self._v_xc.get()
        ac.xpdrS  = self._v_xs.get()

    def _del_ac(self):
        if not self._selected:
            return
        ac = self._selected
        addr = ac.modes_addr
        try:
            self._aircraft.remove(ac)
        except ValueError:
            pass
        self.rx.forget(addr)
        self._colors.pop(addr, None)
        self._selected = None
        self._selected_addr = None
        self._v_name.set("—")
        self._dirty = self._routes_dirty = True
        self._table_dirty = True

    def _reset_positions(self):
        for ac in self._aircraft:
            ac._seg   = 0
            ac._seg_t = 0.0
            ac._lat   = None
            ac._lon   = None
            ac._hdg   = None
            ac._adsb_due.clear()
            ac._pseudo_due.clear()

    # ── Sweep azimuth ─────────────────────────────────────────────────────────

    def _azimuth_now(self):
        t   = time.monotonic()
        rpm = self._rpm_snap
        if rpm != self._sweep_rpm:
            self._sweep_anchor_az = self._azimuth_at(t, self._sweep_rpm)
            self._sweep_anchor_t  = t
            self._sweep_rpm       = rpm
        return self._azimuth_at(t, rpm)

    def _azimuth_at(self, t, rpm):
        return (self._sweep_anchor_az +
                (t - self._sweep_anchor_t) * rpm * 6.0) % 360.0

    # ── Mode selection ────────────────────────────────────────────────────────

    def _current_mode(self):
        for lbl, code in _MODE_LABELS:
            if lbl == self._mode_label_snap:
                return code
        return iff.MODE_3A

    # ── Interrogation ─────────────────────────────────────────────────────────

    def _run_interrogation(self, now, dt):
        prt_s = max(self._prt_s_snap, _MIN_PRT_S)
        self._interro_accum += dt
        n = int(self._interro_accum / prt_s)
        if n <= 0:
            return
        self._interro_accum -= n * prt_s
        n = min(n, _MAX_PRT_PER_FRAME)
        for k in range(n):
            t_k = now - (n - 1 - k) * prt_s
            az  = self._azimuth_at(t_k, self._sweep_rpm)
            self._one_interrogation(az, t_k)

    def _one_interrogation(self, az, t):
        mode = self._current_mode()
        bw   = self._bw_snap
        half_bw = bw / 2.0

        sel_addr = 0
        if mode == iff.MODE_S_SEL:
            sel_addr = self._target_addr_snap or 0
            if sel_addr == 0:
                return  # no target selected for selective mode

        self._prt_no = (self._prt_no + 1) & 0xFFFF

        # Scan tracking: a complete sweep is 360° / (rpm * 6°/s)
        rpm = self._sweep_rpm
        scan_dur = 60.0 / rpm if rpm > 0 else 1e9
        self._scan_no = int(t / scan_dur) if scan_dur > 0 else 0

        # Stage 1 — geometry.  Which aircraft the beam is actually pointing at
        # is the radar's business, so it stays here rather than in the channel.
        illuminated = []
        for ac in self._aircraft:
            if ac.lat is None or ac.lon is None:
                continue
            if not ac.has_xpdr(mode):
                continue

            # Mode S All-Call lockout
            if mode == iff.MODE_S_AC and t < ac._lockout_until:
                continue

            # Mode S Selective: one reply per target per scan
            if mode == iff.MODE_S_SEL:
                if ac.modes_addr != sel_addr:
                    continue
                if ac._last_sel_scan == self._scan_no:
                    continue

            brg, rng = iff.bearing_range_nm(self.c_lat, self.c_lon, ac.lat, ac.lon)
            if abs(iff.angle_diff(brg, az)) > half_bw:
                continue
            illuminated.append((ac, rng))

        if not illuminated:
            return

        # Stage 2 — channel.  Range, horizon, reply probability, round-trip
        # delay.  A delivered frame means the transponder transmitted, so the
        # lockout arms here, before garble: an aircraft that replies and is then
        # garbled still goes quiet, which is what makes acquisition
        # probabilistic instead of instant.
        delivered = []
        for ac, rng in illuminated:
            out = channel.deliver_iff(ac, self.site, mode, t, self._prt_no,
                                      rng_nm=rng, qnh_hpa=self._qnh_snap)
            if mode == iff.MODE_S_AC and out is not None:
                ac._lockout_until = t + _LOCKOUT_S
            if mode == iff.MODE_S_SEL and out is not None:
                ac._last_sel_scan = self._scan_no
            if out is not None:
                frame, t_rx = out
                delivered.append((frame, t_rx, rng))

        if not delivered:
            return

        # Stage 3 — garble.  Only Modes 1/2/3A/C: Mode S replies are addressed
        # and CRC-protected, so they do not garble each other this way.
        if not iff.mode_is_s(mode) and len(delivered) > 1:
            bad = channel.garbled([r for _f, _t, r in delivered])
        else:
            bad = ()

        # Stage 4 — receiver.  Decodes what arrived; measures range from the
        # delay and bearing from the beam.
        for i, (frame, t_rx, _rng) in enumerate(delivered):
            if i in bad:
                self._garble_count += 1
                continue
            brg_meas = channel.measure_bearing(az, self.site)
            if self.rx.rx_iff(frame, t, t_rx, brg_meas) is not None:
                self._table_dirty = True

    # ── Target menu ───────────────────────────────────────────────────────────

    def _refresh_target_menu(self):
        """Rebuild the selective-target dropdown ONLY when its entries change.

        A dirty flag is not enough here: it was set on every received frame, so
        at ~4 ADS-B messages per second per aircraft the menu was deleted and
        re-added several times a second, which tears the list out from under a
        user trying to click an item.  The rebuild is now gated on the content
        itself, and skipped outright while the menu is posted.
        """
        # Sorted by track number so the dropdown order matches the table.
        entries = sorted(self._known_addrs.items(),
                         key=lambda kv: self._tracks.get(kv[0], {}).get("trk_no", 1 << 30))
        labels = [f"TRK{self._tracks.get(a, {}).get('trk_no', 0):03d}  {a:06X}  {how}"
                  for a, how in entries]
        sig = tuple(labels)
        if sig == self._target_menu_sig:
            return

        menu = self._target_om["menu"]
        # Never restructure a menu the user currently has open.
        try:
            if menu.winfo_ismapped():
                return
        except tk.TclError:
            pass

        self._target_menu_sig = sig
        menu.delete(0, "end")
        if not entries:
            menu.add_command(label="(no known address)",
                             command=lambda: self._v_target.set("(no known address)"))
            self._v_target.set("(no known address)")
            self._target_menu_map = {}
            return
        cur = self._v_target.get()
        self._target_menu_map = {}
        for label, (addr, _how) in zip(labels, entries):
            self._target_menu_map[label] = addr
            menu.add_command(label=label, command=lambda l=label: self._v_target.set(l))
        # Only move the selection if what was selected has actually gone away.
        if cur not in self._target_menu_map:
            self._v_target.set(labels[0])

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _view_sig(self):
        return (round(self.c_lat, 6), round(self.c_lon, 6),
                round(self.rng, 3), self._cw, self._ch)

    def _reported_pos(self, trk, now):
        """Where a track was last *reported* to be, as (lat, lon, source).

        Three independent sources, whichever is fresher:
          "iff"     the measured plot — range from round-trip delay, bearing
                    from the beam — converted back to lat/lon for drawing.
          "adsb"    the CPR-resolved reported position.
          "pseudo"  a position decoded from the custom 1090 format.

        Returns None when none is available, which is the correct answer for a
        track that has an address and an altitude but no position yet.
        """
        plot = trk["plot"]
        p_ts = plot["ts"] if plot else -1.0
        if p_ts >= 0 and (now - p_ts) > _FIELD_STALE_S:
            p_ts = -1.0

        rlat = _rx_field(trk, "lat", now)
        rlon = _rx_field(trk, "lon", now)
        if rlat is None or rlon is None:
            lat = lon = which = None
            a_ts = -1.0
        else:
            lat, lon, which = rlat[0], rlon[0], rlat[2]
            a_ts = min(rlat[1], rlon[1])

        if p_ts < 0 and a_ts < 0:
            return None
        if a_ts >= p_ts:
            return lat, lon, which
        # Measured range/bearing back to lat/lon about the radar site.
        rng, brg = plot["rng_nm"], plot["brg_deg"]
        nm_n = rng * math.cos(math.radians(brg))
        nm_e = rng * math.sin(math.radians(brg))
        coslat = max(math.cos(math.radians(self.c_lat)), 1e-6)
        return (self.c_lat + nm_n / 60.0,
                self.c_lon + nm_e / (60.0 * coslat), "iff")

    def _draw(self):
        cv = self.cv
        cx, cy, r = ui.geom(self._cw, self._ch)
        sf = ui.scale_for(self._cw, self._ch)
        now = time.monotonic()

        # Layer 1: background
        sig = self._view_sig()
        if sig != self._bg_sig:
            cv.delete("bg")
            ui.draw_radar_frame(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                sf, tag="bg")
            ui.draw_latlon_grid(cv, cx, cy, r, self.rng, self.c_lat, self.c_lon,
                                tag="bg")
            self._bg_sig = sig
            self._routes_dirty = True
            self._fg_sig = None
            cv.delete("blip")            # geometry moved: rebuild every plot
            self._blip_items.clear()
            self._blip_fade.clear()

        # Layer 2: routes (faint polylines, selected aircraft's path only)
        snap = [(ac, list(ac.waypoints)) for ac in self._aircraft]
        if self._routes_dirty:
            cv.delete("route")
            for ac, wps in snap:
                self._draw_route(cv, ac, wps, cx, cy, r, sf)
            self._routes_dirty = False

        # Layer 3: beam wedge
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

        # Layer 4: plots — drawn from the TRACK STORE, never from ground truth.
        # An aircraft with every transponder off and no ADS-B has no track and
        # therefore no plot, however plainly it is flying about.  Plots step
        # forward as the sweep passes each target rather than gliding, because
        # that is when a reply actually arrives.
        plots = []
        for addr, trk in self._tracks.items():
            rep = self._reported_pos(trk, now)
            if rep is None:
                continue
            lat, lon, src = rep
            pt = self._to_xy(lat, lon)
            if pt is None:
                continue
            age = now - max(trk["iff"].get("last_ts", 0),
                            trk["adsb"].get("last_ts", 0),
                            trk["pseudo"].get("last_ts", 0))
            plots.append((addr, trk, pt, age, src))

        # 6.5 — one structural signature PER TRACK, not one for the whole
        # picture, and no age term in it.  Previously the signature was global
        # and carried a 0.125 s age bucket, so every blip was deleted and
        # recreated ~8x/s; a single global signature would also rebuild all
        # eight tracks whenever any one of them moved.  Now a track's items are
        # rebuilt only when that track's own geometry or text changes, and the
        # colour fade is an itemconfig on the cached ids.
        live = set()
        for addr, trk, (x, y), age, src in plots:
            live.add(addr)
            _tk = _rx_field(trk, "track_deg", now)
            hdg = None if _tk is None else _tk[0]
            _cl = _rx_field(trk, "call", now)
            call = (_fld(trk["iff"], "call", now)
                    or (_cl[0] if _cl else None)
                    or f"TRK{trk['trk_no']:03d}")
            _al = _rx_field(trk, "alt_ft", now)
            alt = _al[0] if _al else _fld(trk["iff"], "alt_ft", now)
            bits = [fmt_alt(alt)] if alt is not None else []
            if hdg is not None:
                bits.append(f"{self._mag(hdg):03.0f}°M")
            data = "  ".join(bits)
            sig = (round(x, 1), round(y, 1),
                   None if hdg is None else round(hdg, 1), call, data)

            cached = self._blip_items.get(addr)
            if cached is None or cached[0] != sig:
                if cached is not None:
                    for it in cached[1]:
                        cv.delete(it)
                items = []
                if hdg is not None:
                    ui.draw_blip(cv, x, y, math.radians(hdg), ui.FG, sf, tag="blip")
                    items.append(cv.find_withtag("blip")[-1])
                else:
                    rad = ui.BLIP_SZ * sf
                    items.append(cv.create_oval(x - rad, y - rad, x + rad, y + rad,
                                                fill=ui.FG, outline="", tags="blip"))
                # Label from decoded data only: until a callsign is received the
                # track is known by its number, which is how it really works.
                items.append(cv.create_text(
                    x + ui.LBL_DX * sf, y - ui.LBL_DY * sf,
                    text=call, fill=ui.FG,
                    font=ui.sfont(ui.PT_MD, sf, bold=True),
                    anchor="w", tags="blip"))
                if data:
                    items.append(cv.create_text(
                        x + ui.LBL_DX * sf, y - round(2 * ui.SCALE * sf),
                        text=data, fill=ui.DIM, font=ui.sfont(ui.PT_SM, sf),
                        anchor="w", tags="blip"))
                self._blip_items[addr] = (sig, items)
                self._blip_fade.pop(addr, None)

            # Colour fade in place, quantised so it is not reissued every frame.
            items = self._blip_items[addr][1]
            frac = min(age / _BLIP_DECAY_S, 1.0)
            bucket = int(frac * 16)
            if self._blip_fade.get(addr) != bucket:
                self._blip_fade[addr] = bucket
                faded = ui.blend(trk.get("color") or ui.FG, ui.DIM, frac)
                cv.itemconfig(items[0], fill=faded)
                cv.itemconfig(items[1], fill=faded)

        # Drop items for tracks that no longer have a plot.
        for addr in [a for a in self._blip_items if a not in live]:
            for it in self._blip_items.pop(addr)[1]:
                cv.delete(it)
            self._blip_fade.pop(addr, None)

        # Truth overlay — debug only, off by default, drawn dim so it can never
        # be mistaken for a plot.  Redrawn per frame since it follows continuous
        # ground truth rather than stepped plots.
        cv.delete("truth")
        if self._truth_on:
            for ac in self._aircraft:
                if ac.lat is None or ac.lon is None:
                    continue
                pt = self._to_xy(ac.lat, ac.lon)
                if pt is None:
                    continue
                x, y = pt
                d = round(2 * ui.SCALE * sf)
                cv.create_line(x - d, y - d, x + d, y + d,
                               fill=_TRUTH_COL, tags=("blip", "truth"))
                cv.create_line(x - d, y + d, x + d, y - d,
                               fill=_TRUTH_COL, tags=("blip", "truth"))

        # Stale track cleanup — drop tracks with no recent IFF or ADS-B data
        if self.rx.prune(now, _BLIP_DECAY_S):
            self._table_dirty = True

        # Layer 5: cursor
        cv.delete("cur")
        if self._cursor_on and self._cursor:
            x, y, lat, lon = self._cursor
            arm = round(_CURSOR_ARM_PX * sf)
            cv.create_line(x, y - arm, x, y + arm, fill=ui.SEP, tags="cur")
            cv.create_line(x - arm, y, x + arm, y, fill=ui.SEP, tags="cur")
            o = round(6 * ui.SCALE * sf)
            cv.create_text(x + o, y - o,
                           text=f"{lat:+.4f}  {lon:+.4f}",
                           fill=ui.FG, font=ui.sfont(ui.PT_SM, sf),
                           anchor="sw", tags="cur")

    def _draw_route(self, cv, ac, wps, cx, cy, r, sf):
        # Routes are authoring input, not received data, so drawing them from
        # the aircraft is correct — but the colour comes from the same
        # per-address table the plots use, so a route and its plot match.
        leg = self._color_for(ac.modes_addr) if ac is self._selected else "#555555"
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

    # ── Track table ───────────────────────────────────────────────────────────

    _COL_FMT = ("{trk:>3} {call:<8} {addr:>6} {sqwk:>4} {m1:>2} {m2:>4} "
            "{alt:>5} {rng:>5} {brg:>3} {age:>4}")

    def _table_row(self, *, trk, call, addr, sqwk, m1, m2, alt, rng, brg, age):
        return self._COL_FMT.format(trk=trk, call=call[:8], addr=addr, sqwk=sqwk,
                                    m1=m1, m2=m2, alt=alt, rng=rng, brg=brg, age=age)

    def _flush_table(self):
        now = time.monotonic()
        rows = []

        # 6.1 — two aircraft sharing an address collapse into one track, which
        # on screen looks like a single aircraft teleporting.  Alert instead.
        seen = set()
        dupes = set()
        for a in self._aircraft:
            if a.modes_addr in seen:
                dupes.add(a.modes_addr)
            seen.add(a.modes_addr)
        self._dupe_addrs = dupes
        if dupes:
            self._v_status.set("⚠ duplicate address: "
                               + " ".join(f"{a:06X}" for a in sorted(dupes)))
            self._status_lbl.config(fg=_DWELL_LOW_COL)

        for addr, trk in self._tracks.items():
            di, da, dp = trk["iff"], trk["adsb"], trk["plot"]
            age = now - max(di.get("last_ts", 0), da.get("last_ts", 0),
                            trk["pseudo"].get("last_ts", 0))
            if age > _BLIP_DECAY_S:
                continue

            # Range/bearing: the measured IFF plot, or the ADS-B reported
            # position — never recomputed from ground truth.
            rng = brg = None
            if dp and (now - dp["ts"]) <= _FIELD_STALE_S:
                rng, brg = dp["rng_nm"], dp["brg_deg"]
            else:
                _la, _lo = _rx_field(trk, "lat", now), _rx_field(trk, "lon", now)
                lat = _la[0] if _la else None
                lon = _lo[0] if _lo else None
                if lat is not None and lon is not None:
                    brg, rng = iff.bearing_range_nm(self.c_lat, self.c_lon, lat, lon)

            # Each field ages independently.
            # IFF fields first, then whichever 1090 format is fresher.
            def _any(key):
                v = _fld(di, key, now)
                if v is not None:
                    return v
                r = _rx_field(trk, key, now)
                return r[0] if r else None

            sqwk = _any("sqwk")
            m1   = _any("m1")
            m2   = _any("m2")
            _al  = _rx_field(trk, "alt_ft", now)
            alt  = _al[0] if _al else _fld(di, "alt_ft", now)
            # Callsign: prefer IFF Mode S Selective BDS 2,0 over the 1090 stream.
            call = _any("call") or "—"
            # ADDR shows once any source has actually delivered it — a Mode S
            # reply, or a custom format that carries the address.
            ms_addr = _fld(di, "modes_addr", now)
            if ms_addr is None and self.rx.known_addrs.get(addr) == "C" \
                    and trk["pseudo"].get("last_ts"):
                if (now - trk["pseudo"]["last_ts"]) <= _FIELD_STALE_S:
                    ms_addr = addr

            # 5.7 — special-purpose codes and the ident pulse get the row
            # highlighted, since they are the whole reason to notice a row.
            emerg = iff.emergency_label(sqwk) if sqwk is not None else None
            spi = any(a.modes_addr == addr and now < a.spi_until
                      for a in self._aircraft)

            rows.append((rng if rng is not None else 1e9, addr, {
                "trk":  f"{trk['trk_no']:03d}",
                "call": call,
                "addr": f"{ms_addr:06X}" if ms_addr is not None else "—",
                "sqwk": f"{sqwk:04o}" if sqwk is not None else "—",
                "m1":   f"{m1:02o}"   if m1   is not None else "—",
                "m2":   f"{m2:04o}"   if m2   is not None else "—",
                "alt":  fmt_alt(alt),
                "rng":  f"{rng:5.1f}" if rng is not None else "    —",
                "brg":  f"{brg:3.0f}" if brg is not None else "  —",
                "age":  f"{age:4.1f}",
            }, emerg, spi))

        rows.sort(key=lambda t: t[0])

        self._tbl.config(state=tk.NORMAL)
        self._tbl.delete("1.0", tk.END)
        self._row_addrs = []
        for _rng, addr, col, emerg, spi in rows:
            if addr == self._selected_addr:
                tag = "sel"
            elif emerg:
                tag = "emerg"
            elif spi:
                tag = "spi"
            else:
                tag = "row"
            suffix = f"  {emerg}" if emerg else ("  IDENT" if spi else "")
            self._tbl.insert(tk.END, self._table_row(**col) + suffix + "\n", tag)
            self._row_addrs.append(addr)
        if not rows:
            self._tbl.insert(tk.END, "  (no tracks)\n", "row")
        self._tbl.config(state=tk.DISABLED)

        self._refresh_target_menu()
        self._refresh_detail()
        # Set last: the ADS-B rows above are built inside this same pass, so
        # clearing the flag earlier would drop an update that arrived mid-build.
        self._table_dirty = False

    def _refresh_detail(self):
        """Phase 7 — a decoded readout of the selected track, replacing the raw
        hex dump.  Shows every accumulated IFF mode field, the ADS-B state, and
        the measured plot, each with its own age, so it is obvious which values
        are live and which are merely the last thing ever heard.

        (The plan refers this out to phase-7-decoded-pane.md, which is not in
        the repo; this implements the one-line brief given in the plan itself.)
        """
        addr = self._selected_addr
        t = self._detail
        now = time.monotonic()
        t.config(state=tk.NORMAL)
        t.delete("1.0", tk.END)

        if addr is None:
            t.insert(tk.END, "(click a track on the radar or in the table)")
            t.config(state=tk.DISABLED)
            return

        trk = self._tracks.get(addr)
        if trk is None:
            t.insert(tk.END, f"{addr:06X}\n", "hdr")
            t.insert(tk.END, "\n(track decayed — no frames received)", "stale")
            t.config(state=tk.DISABLED)
            return

        di, da, dp = trk["iff"], trk["adsb"], trk["plot"]

        def field(label, key, src, fmt=str, extra=""):
            """One decoded field line, tagged live or stale by its own age."""
            ent = src.get(key)
            if ent is None:
                t.insert(tk.END, f"  {label:<5} {'—':<12}\n", "stale")
                return
            val, ts = ent
            age = now - ts
            stale = age > _FIELD_STALE_S
            line = f"  {label:<5} {fmt(val):<12} {age:5.1f}s{extra}\n"
            t.insert(tk.END, line, "stale" if stale else "row")

        how = self._known_addrs.get(addr)
        prov = {"S": "Mode S reply", "A": "ADS-B",
                "C": "custom 1090 format"}.get(how, "not yet learned")
        t.insert(tk.END, f"TRK{trk['trk_no']:03d}   {addr:06X}\n", "hdr")
        t.insert(tk.END, f"address via {prov}\n\n", "stale")

        # ── IFF: every field this radar has managed to accumulate ──
        if di.get("last_ts") is None:
            t.insert(tk.END, "IFF   (no reply yet)\n\n", "stale")
        else:
            mode_name = iff.MODE_NAMES.get(di.get("last_mode"), "?")
            t.insert(tk.END, f"IFF   last {mode_name}  prt {di.get('last_prt', '?')}"
                             f"  {now - di['last_ts']:.1f}s\n", "hdr")
            field("M1", "m1", di, lambda v: f"{v:02o}")
            field("M2", "m2", di, lambda v: f"{v:04o}")
            sq = di.get("sqwk")
            emerg = iff.emergency_label(sq[0]) if sq else None
            field("SQWK", "sqwk", di, lambda v: f"{v:04o}",
                  extra=f"  {emerg}" if emerg else "")
            field("ALT", "alt_ft", di, lambda v: f"{fmt_alt(v)} (P)")
            field("ADDR", "modes_addr", di, lambda v: f"{v:06X}")
            field("CALL", "call", di)
            t.insert(tk.END, "\n")

        # ── Measured plot: range from delay, bearing from the beam ──
        if not dp:
            t.insert(tk.END, "PLOT  (no measured plot)\n\n", "stale")
        else:
            age = now - dp["ts"]
            tag = "stale" if age > _FIELD_STALE_S else "row"
            t.insert(tk.END, f"PLOT  measured  {age:.1f}s\n", "hdr")
            t.insert(tk.END, f"  RNG   {dp['rng_nm']:.3f} nm"
                             f"  (1/{int(1 / iff.RANGE_QUANT_NM)} nm cell)\n", tag)
            t.insert(tk.END, f"  BRG   {dp['brg_deg']:06.2f}°T"
                             f"  {self._mag(dp['brg_deg']):06.2f}°M\n", tag)
            t.insert(tk.END, "\n")

        # ── ADS-B state ──
        if da.get("last_ts") is None:
            t.insert(tk.END, "ADSB  (no message yet)\n", "stale")
        else:
            t.insert(tk.END, f"ADSB  last {da.get('last_type', '?')}"
                             f"  {now - da['last_ts']:.1f}s\n", "hdr")
            field("CALL", "call", da)
            field("LAT", "lat", da, lambda v: f"{v:+.5f}")
            field("LON", "lon", da, lambda v: f"{v:+.5f}")
            field("ALT", "alt_ft", da, lambda v: f"{fmt_alt(v)} (P)")
            field("TRK", "track_deg", da, lambda v: f"{v:05.1f}°")
            field("GS", "speed_kt", da, lambda v: f"{v:.0f} kt")
            field("V/S", "vrate_fpm", da, lambda v: f"{v:+.0f} fpm")
            if da.get("lat") is None:
                t.insert(tk.END, "  CPR   awaiting even/odd pair\n", "stale")

        # ── Custom 1090 format ──
        dc = trk["pseudo"]
        if dc.get("last_ts") is not None:
            t.insert(tk.END, f"\nC1090 last {dc.get('last_type', '?')}"
                             f"  {now - dc['last_ts']:.1f}s\n", "hdr")
            spec = dc.get("_spec")
            vals = dc.get("_decoded") or {}
            dts = dc.get("_decoded_ts", now)
            stale = (now - dts) > _FIELD_STALE_S
            if self.fmt is not None and self.fmt.addr_field and "address" in vals:
                t.insert(tk.END, f"  {'ADDR':<5} {vals['address']:06X}"
                                 f"       {now - dts:5.1f}s\n",
                         "stale" if stale else "row")
            if spec is not None:
                for fld in spec.fields:
                    shown = fld.format(vals.get(fld.name))
                    t.insert(tk.END, f"  {fld.name[:5]:<5} {shown:<12} "
                                     f"{now - dts:5.1f}s\n",
                             "stale" if stale else "row")
            if "last_raw" in dc:
                t.insert(tk.END, "  " + str(dc["last_raw"]) + "\n", "stale")

        # Raw frames stay available, just no longer the whole story.
        if "last_raw" in di:
            t.insert(tk.END, "\nIFF raw\n", "hdr")
            t.insert(tk.END, iff.format_hex(di["last_raw"]) + "\n", "stale")
        if "last_raw" in da:
            h = da["last_raw"]
            t.insert(tk.END, "\nADSB raw\n", "hdr")
            t.insert(tk.END, " ".join(h[i:i + 2] for i in range(0, len(h), 2)),
                     "stale")

        t.config(state=tk.DISABLED)

    # ── Selection from canvas / table ─────────────────────────────────────────

    def _on_table_click(self, ev):
        idx = self._tbl.index(f"@{ev.x},{ev.y}")
        line_no = int(idx.split(".")[0])
        if 1 <= line_no <= len(self._row_addrs):
            addr = self._row_addrs[line_no - 1]
            ac = next((a for a in self._aircraft if a.modes_addr == addr), None)
            if ac:
                self._select(ac)
            else:
                # A track with no live aircraft behind it (decaying) can still
                # be selected so its last reply stays readable.
                self._selected_addr = addr
                self._table_dirty = True

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        self._frame()
        self.after(_FRAME_MS, self._loop)

    def _frame(self):
        """One simulation frame.  Split out from _loop so tests can step the
        model without the Tk timer owning the clock."""
        now = time.monotonic()
        dt = now - self._tick
        self._tick = now

        # 1. Physics: step all aircraft
        for ac in self._aircraft:
            ac.step(dt)

        # 2. Refresh Tk var snapshots
        try:
            self._rpm_snap        = self._v_rpm.get()
            self._bw_snap         = float(self._v_bw.get())
            self._prt_s_snap      = self._v_prt.get() * 1e-6
            self._qnh_snap        = float(self._v_qnh.get())
            self._mode_label_snap = self._v_mode.get()
            self._target_addr_snap = self._target_menu_map.get(self._v_target.get())
            rpm = self._rpm_snap
            bw  = self._bw_snap
            prt = self._prt_s_snap
            # Interrogations per beam dwell.  This is a real SSR constraint,
            # not a bug to paper over: a 3° beam at 15 rpm with a 30 ms PRT
            # genuinely yields ~1.1 hits, which is why PRF and rotation rate
            # are matched in practice.  Surfaced, never compensated for by
            # widening the beam-inclusion test.
            hits = bw / (rpm * 6.0 * prt) if prt > 0 else 0.0
            self._v_dwell.set(f"hits/dwell {hits:.1f}")
            self._dwell_lbl.config(fg=_DWELL_LOW_COL if hits < _DWELL_MIN_HITS
                                   else ui.FG_DIM)

            if self._dupe_addrs:
                pass          # _flush_table owns the message in this case
            elif self._current_mode() == iff.MODE_S_SEL and not self._target_addr_snap:
                self._v_status.set("⚠ no target selected")
                self._status_lbl.config(fg=_DWELL_LOW_COL)
            else:
                self._v_status.set("")
        except (tk.TclError, ValueError):
            # An IntVar/DoubleVar mid-edit can hold a non-numeric string; keep
            # the previous snapshot rather than swallowing every error class.
            pass

        # 3. Run interrogation for this frame
        self._run_interrogation(now, dt)

        # 4. 1090 MHz: aircraft emit, the channel drops some, the receiver
        # decodes.  Standard ADS-B and the custom format share this path
        # because they share the physical layer — same range, same horizon,
        # same per-frame loss.  Which of them is transmitted is the mode.
        mode = self._tx_mode
        for ac in self._aircraft:
            if mode in (pseudo1090.MODE_STANDARD, pseudo1090.MODE_BOTH):
                out = ac.adsb_frame(now, self._qnh_snap)
                if out is not None:
                    label, frame = out
                    frame = channel.deliver_adsb(ac, self.site, frame, now)
                    if frame is None:
                        self._log_1090("ADSB", label, None, lost=True)
                    else:
                        ok = self.rx.rx_adsb(frame, now) is not None
                        self._log_1090("ADSB", label, frame, decoded=ok)
                        self._table_dirty = True

            if mode in (pseudo1090.MODE_CUSTOM, pseudo1090.MODE_BOTH) \
                    and self.fmt is not None:
                out = ac.pseudo_frame(now, self.fmt)
                if out is not None:
                    name, frame = out
                    frame = channel.deliver_adsb(ac, self.site, frame, now)
                    if frame is None:
                        self._log_1090("CUSTOM", name, None, lost=True)
                    else:
                        ok = self.rx.rx_pseudo(frame, now, self.fmt) is not None
                        self._log_1090("CUSTOM", name, frame, decoded=ok)
                        self._table_dirty = True

        # 5. Refresh aircraft listbox
        self._refresh_list()

        # 6. Draw PPI
        self._draw()

        # 7. Flush track table (~5 Hz or when dirty)
        if self._table_dirty or (now - self._table_last) >= 0.2:
            self._flush_table()
            self._flush_log()
            self._table_last = now

    # ── Fullscreen ────────────────────────────────────────────────────────────

    def _toggle_fullscreen(self, _ev=None):
        self._fullscreen = not self._fullscreen
        self.attributes("-fullscreen", self._fullscreen)

    def _exit_fullscreen(self, _ev=None):
        self._fullscreen = False
        self.attributes("-fullscreen", False)


    # ── Entry point ───────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="IFF Radar + Aircraft Simulator (merged)",
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__)
    p.add_argument("--centre", metavar="LAT,LON", default=None)
    p.add_argument("--range",  type=float, default=200.0)
    p.add_argument("--declination", type=float, default=0.0)
    p.add_argument("--format-cfg", metavar="PATH", default=None,
                   help="custom 1090 message format "
                        f"(default: {os.path.basename(pseudo1090.DEFAULT_CFG)})")
    args = p.parse_args()
    lat, lon = 51.477, -0.461
    if args.centre:
        lat, lon = map(float, args.centre.split(","))
    CombinedApp(lat, lon, args.range, args.declination,
                cfg_path=args.format_cfg).mainloop()


if __name__ == "__main__":
    main()
