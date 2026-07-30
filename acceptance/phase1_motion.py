"""Phase 1 — motion, lifecycle, dead locks.

Run directly, or via run_all.py.
"""

import sys
import time

from harness import (Clock, Suite, add_ac, frames, iff, make_app, source)
import simulator as S

suite = Suite("Phase 1 — motion, lifecycle, dead locks")
chk = suite.chk

# ── 1.8 no threading ─────────────────────────────────────────────────────────
src = source("simulator.py")
chk("1.8 no threading import", "threading" not in src)
chk("1.8 no self._lock", "self._lock " not in src and "self._lock." not in src)
chk("1.8 no _tracks_lock", "_tracks_lock" not in src)

# ── 1.6 fmt_alt ──────────────────────────────────────────────────────────────
chk("1.6 fmt_alt(None)", S.fmt_alt(None) == "—", S.fmt_alt(None))
chk("1.6 fmt_alt(-1000) plain", S.fmt_alt(-1000) == "-1000", S.fmt_alt(-1000))
chk("1.6 fmt_alt(-550) rounds", S.fmt_alt(-550) == "-600", S.fmt_alt(-550))
chk("1.6 fmt_alt(3500) plain", S.fmt_alt(3500) == "3500", S.fmt_alt(3500))
chk("1.6 fmt_alt(35000) FL", S.fmt_alt(35000) == "FL350", S.fmt_alt(35000))
chk("1.6 fmt_alt(18000) FL", S.fmt_alt(18000) == "FL180", S.fmt_alt(18000))

# ── 1.7 _MODE_FLAG keyed on constants ────────────────────────────────────────
ac = S.SimAircraft()
chk("1.7 has_xpdr(MODE_1)", ac.has_xpdr(iff.MODE_1) is True)
chk("1.7 has_xpdr(MODE_S_SEL)", ac.has_xpdr(iff.MODE_S_SEL) is True)
try:
    ac.has_xpdr(99)
    chk("1.7 unknown mode raises", False)
except ValueError:
    chk("1.7 unknown mode raises", True)

# ── 1.1 loop mode flies the closing leg ──────────────────────────────────────
# Deliberately asymmetric quad so the closing leg wps[-1]->wps[0] is NOT a
# cardinal bearing — a cardinal closing leg would make the heading check
# pass trivially.
sq = [(52.0, -0.461), (52.0, 1.0), (51.2, 1.3), (51.0, 0.2)]
_closing_brg = iff.bearing_range_nm(sq[-1][0], sq[-1][1], sq[0][0], sq[0][1])[0]
assert 5.0 < _closing_brg < 355.0 and abs(_closing_brg - 90) > 5, _closing_brg
clock = Clock(dt=0.05)
clock.install()
try:
    app = make_app(clock)
    a = add_ac(app, sq, speed_kt=3600)   # ~234 nm lap -> 234 s
    a.loop = True
    max_jump = 0.0
    prev = None
    segs_seen = set()
    hdg_on_closing = []
    # Fly well past the end of the open path so the closing leg is exercised.
    for _ in range(6000):          # 300 s at dt=0.05
        clock.advance(clock.dt)
        a.step(clock.dt)
        segs_seen.add(a._seg)
        if a._seg == 3:
            hdg_on_closing.append(a.course())
        p = (a.lat, a.lon)
        if prev:
            d = iff.bearing_range_nm(prev[0], prev[1], p[0], p[1])[1]
            max_jump = max(max_jump, d)
        prev = p
    # 3600 kt * 0.05 s = 0.05 nm per step; allow 3x for waypoint corners.
    chk("1.1 loop never teleports", max_jump < 0.15, f"max step {max_jump:.4f} nm")
    chk("1.1 closing leg (seg 3) is actually flown", 3 in segs_seen,
        f"segs seen {sorted(segs_seen)}")
    chk("1.1 loop wraps back to seg 0", segs_seen >= {0, 1, 2, 3},
        f"segs seen {sorted(segs_seen)}")
    # Course while on the closing leg must match the drawn wps[-1]->wps[0]
    # line.  (heading() is rate-limited since 5.5 and lags it in turns.)
    worst = max(abs(iff.angle_diff(h, _closing_brg)) for h in hdg_on_closing)
    chk("1.1 closing-leg course matches drawn line", worst < 1e-9,
        f"non-cardinal brg {_closing_brg:.2f}, worst err {worst:.2e}, "
        f"{len(hdg_on_closing)} samples")

    # ── 1.2 speed 0 freezes in place ─────────────────────────────────────────
    b = add_ac(app, sq, speed_kt=600)
    for _ in range(200):
        b.step(0.05)
    frozen = (b.lat, b.lon)
    chk("1.2 moved off wp0 before freeze",
        iff.bearing_range_nm(frozen[0], frozen[1], sq[0][0], sq[0][1])[1] > 1.0)
    b.speed_kt = 0
    for _ in range(200):
        b.step(0.05)
    chk("1.2 speed 0 holds position", (b.lat, b.lon) == frozen,
        f"{frozen} -> {(b.lat, b.lon)}")
    b.speed_kt = 600
    b.step(0.05)
    chk("1.2 resumes from frozen point",
        iff.bearing_range_nm(frozen[0], frozen[1], b.lat, b.lon)[1] < 0.03)

    # ── 1.3 duplicate waypoints do not stall ─────────────────────────────────
    dup = [(52.0, -0.461), (52.0, -0.461), (52.0, 1.0), (52.0, 1.0), (51.0, 1.0)]
    c = add_ac(app, dup, speed_kt=600)
    start = None
    for i in range(2000):
        c.step(0.05)
        if i == 0:
            start = (c.lat, c.lon)
    chk("1.3 passes through duplicate wps",
        iff.bearing_range_nm(start[0], start[1], c.lat, c.lon)[1] > 10.0,
        f"travelled {iff.bearing_range_nm(start[0], start[1], c.lat, c.lon)[1]:.1f} nm")

    # all-degenerate path must not spin
    d = add_ac(app, [(52.0, 0.0)] * 5, speed_kt=600)
    t0 = time.perf_counter_ns()
    for _ in range(100):
        d.step(0.05)
    el = (time.perf_counter_ns() - t0) / 1e6
    chk("1.3 all-degenerate path is bounded", el < 200, f"{el:.1f} ms for 100 steps")

    # ── 1.4 track stores drain ───────────────────────────────────────────────
    app2 = make_app(clock)
    for _ in range(20):
        add_ac(app2, [(51.6, -0.4), (51.7, -0.3)], speed_kt=300)
    app2._v_mode.set("Mode S All-Call")
    frames(app2, clock, 40)
    seeded = len(app2._tracks)
    chk("1.4 tracks were seeded", seeded > 0, f"{seeded} tracks")
    for ac in list(app2._aircraft):
        app2._selected = ac
        app2._del_ac()
    chk("1.4 aircraft list empty", not app2._aircraft)
    chk("1.4 _tracks empty after delete", not app2._tracks, str(list(app2._tracks)))
    chk("1.4 _known_addrs empty", not app2._known_addrs, str(list(app2._known_addrs)))

    # stale decay path: seed, then let 5 s pass with no aircraft replying
    app3 = make_app(clock)
    for _ in range(5):
        add_ac(app3, [(51.6, -0.4), (51.7, -0.3)], speed_kt=300)
    frames(app3, clock, 40)
    chk("1.4 stale-decay seeded", len(app3._tracks) > 0)
    app3._aircraft.clear()
    clock.advance(5.0)
    frames(app3, clock, 3)
    chk("1.4 _tracks decay to empty", not app3._tracks, str(list(app3._tracks)))

    # ── 1.5 minsize is sane ──────────────────────────────────────────────────
    mw, mh = app.minsize()
    expect_w = S.ui.CANVAS_SZ + S._PANEL_W + S._REPLY_W + round(20 * S.ui.SCALE)
    chk("1.5 minsize not double-scaled", mw == expect_w, f"{mw} vs {expect_w}")

    # ── 1.9 nits ─────────────────────────────────────────────────────────────
    chk("1.9 _MIN_PRT_S matches slider floor", S._MIN_PRT_S == 5000e-6)
    chk("1.9 _DRAW_SPACING_PX used", "_DRAW_SPACING_PX * ui.scale_for" in src)
    # The detail pane must print the PRT the stored frame was encoded under,
    # not whatever self._prt_no happens to be now.
    _detail_body = src.split("def _refresh_detail")[1].split("# ── Selection")[0]
    chk("1.9 _refresh_detail prints stored prt",
        "last_prt" in _detail_body and "self._prt_no" not in _detail_body)
    chk("1.9 _reset_positions resets adsb", "ac._adsb_due.clear()" in src)
    chk("1.9 _azimuth_now single monotonic call",
        src.count("t   = time.monotonic()") == 1)
finally:
    clock.restore()

sys.exit(suite.report())
