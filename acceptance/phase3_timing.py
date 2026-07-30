"""Phase 3 — interrogation timing.

Run directly, or via run_all.py.
"""

import sys

from harness import (Clock, Suite, add_ac, frames, iff, make_app, source)
import simulator as S

suite = Suite("Phase 3 — interrogation timing")
chk = suite.chk

def count_interrogations(app, clock, n_frames):
    """Count _one_interrogation calls and capture (az, t) for each."""
    calls = []
    orig = app._one_interrogation
    app._one_interrogation = lambda az, t: (calls.append((az, t)), orig(az, t))[1]
    frames(app, clock, n_frames, draw=False)
    app._one_interrogation = orig
    return calls


clock = Clock(dt=0.05)
clock.install()
try:
    # ── 3.1 PRT is not clamped to the frame rate ─────────────────────────────
    app = make_app(clock)
    add_ac(app, [(51.6, -0.4), (51.7, -0.3)], speed_kt=300)
    app._v_prt.set(5000)                 # 5000 us -> 200 Hz
    frames(app, clock, 2, draw=False)    # let the snapshot pick the slider up
    calls = count_interrogations(app, clock, 20)   # 1.0 simulated second
    rate = len(calls) / (20 * clock.dt)
    chk("3.1 5000 us PRT gives ~200 interrogations/s", 190 <= rate <= 210,
        f"{rate:.0f}/s (frame rate would be 20/s)")

    app._v_prt.set(30000)                # default 30 ms -> 33.3 Hz
    frames(app, clock, 2, draw=False)
    calls = count_interrogations(app, clock, 20)
    rate = len(calls) / (20 * clock.dt)
    chk("3.1 30000 us PRT gives ~33 interrogations/s", 30 <= rate <= 36, f"{rate:.0f}/s")

    # each PRT must carry the azimuth it actually fired at, spread across the frame
    app._v_prt.set(5000)
    frames(app, clock, 2, draw=False)
    calls = count_interrogations(app, clock, 1)
    azs = [az for az, _t in calls]
    ts  = [t for _az, t in calls]
    chk("3.1 many PRTs inside one frame", len(calls) >= 9, f"{len(calls)} in one frame")
    chk("3.1 each PRT has its own timestamp", len(set(ts)) == len(ts))
    chk("3.1 azimuth advances within the frame", len(set(azs)) == len(azs),
        f"{len(set(azs))} distinct of {len(azs)}")
    # 15 rpm = 90 deg/s; 5 ms PRT -> 0.45 deg between interrogations
    if len(azs) >= 2:
        step = abs(iff.angle_diff(azs[1], azs[0]))
        chk("3.1 azimuth step matches rpm * prt", abs(step - 0.45) < 0.02,
            f"{step:.3f} deg, expected 0.45")

    # the per-frame cap holds
    app._v_prt.set(5000)
    frames(app, clock, 2, draw=False)
    clock.advance(30.0)                  # a huge stall: 6000 PRTs' worth
    calls = count_interrogations(app, clock, 1)
    chk("3.1 per-frame PRT count is capped", len(calls) <= S._MAX_PRT_PER_FRAME,
        f"{len(calls)} <= {S._MAX_PRT_PER_FRAME}")

    # ── 3.2 hits-per-dwell readout ───────────────────────────────────────────
    app2 = make_app(clock)
    app2._v_rpm.set(15); app2._v_bw.set(3.0); app2._v_prt.set(30000)
    frames(app2, clock, 2, draw=False)
    chk("3.2 defaults read ~1.1 hits/dwell", app2._v_dwell.get() == "hits/dwell 1.1",
        app2._v_dwell.get())
    chk("3.2 low hit count is red", app2._dwell_lbl.cget("fg") == S._DWELL_LOW_COL,
        app2._dwell_lbl.cget("fg"))

    # tracks all three sliders live
    app2._v_prt.set(5000)                # 3 / (15*6*0.005) = 6.67
    frames(app2, clock, 2, draw=False)
    chk("3.2 tracks the prt slider", app2._v_dwell.get() == "hits/dwell 6.7",
        app2._v_dwell.get())
    chk("3.2 healthy hit count is dim", app2._dwell_lbl.cget("fg") == S.ui.FG_DIM,
        app2._dwell_lbl.cget("fg"))
    app2._v_bw.set(6.0)                  # 6 / (15*6*0.005) = 13.3
    frames(app2, clock, 2, draw=False)
    chk("3.2 tracks the bw slider", app2._v_dwell.get() == "hits/dwell 13.3",
        app2._v_dwell.get())
    app2._v_rpm.set(30)                  # 6 / (30*6*0.005) = 6.67
    frames(app2, clock, 2, draw=False)
    chk("3.2 tracks the rpm slider", app2._v_dwell.get() == "hits/dwell 6.7",
        app2._v_dwell.get())

    # beam inclusion must NOT be widened to compensate
    body = source("simulator.py")
    body = body.split("def _one_interrogation")[1].split("def _track_for")[0]
    chk("3.2 beam test uses half the beam width unmodified",
        "abs(iff.angle_diff(brg, az)) > half_bw" in body)

    # ── 3.3 Mode S all-call lockout ──────────────────────────────────────────
    app3 = make_app(clock)
    ac = add_ac(app3, [(51.6, -0.4), (51.7, -0.3)], speed_kt=0)
    app3._v_mode.set("Mode S All-Call")
    replies = []
    orig = app3.rx.rx_iff
    app3.rx.rx_iff = lambda *a: (replies.append(a[1]), orig(*a))[1]
    frames(app3, clock, 200, draw=False)          # 10 s, 2.5 scans
    chk("3.3 all-call replies exactly once inside the lockout window",
        len(replies) == 1, f"{len(replies)} replies in 10 s")
    chk("3.3 lockout was armed", ac._lockout_until > 0)
    frames(app3, clock, 500, draw=False)          # well past the 18 s lockout
    chk("3.3 replies again after the lockout expires", len(replies) >= 2,
        f"{len(replies)} replies by 35 s")
    app3.rx.rx_iff = orig

    # ── 3.3 selective roll-call: one reply per target per scan ───────────────
    app4 = make_app(clock)
    ac4 = add_ac(app4, [(51.6, -0.4), (51.7, -0.3)], speed_kt=0)
    # seed the address list so a target can be selected
    app4._v_mode.set("Mode S All-Call")
    frames(app4, clock, 120, draw=False)
    app4._v_mode.set("Mode S Selective")
    label = next(iter(app4._target_menu_map))
    app4._v_target.set(label)
    ac4._lockout_until = 0.0
    replies = []
    orig4 = app4.rx.rx_iff
    app4.rx.rx_iff = lambda *a: (replies.append(a[1]), orig4(*a))[1]
    frames(app4, clock, 400, draw=False)          # 20 s = 5 scans at 15 rpm
    scans = 20.0 / (60.0 / 15)
    chk("3.3 selective yields <= 1 reply per scan",
        len(replies) <= scans + 1, f"{len(replies)} replies over {scans:.0f} scans")
    chk("3.3 selective still acquires the target", len(replies) >= 1,
        f"{len(replies)} replies")
    app4.rx.rx_iff = orig4

    # ── 3.3 selective with no target is called out in the panel ──────────────
    # No aircraft at all, so nothing seeds the address list from either source
    # — this is precisely when selective mode would fall back to addr 0 and go
    # silently dead.  (With any aircraft present, ADS-B seeds the menu and the
    # dropdown auto-selects, so there IS always a target: see 2.3.)
    app5 = make_app(clock)
    app5._v_mode.set("Mode S Selective")
    frames(app5, clock, 4, draw=False)
    chk("3.3 no addresses known with an empty airspace", not app5._known_addrs)
    chk("3.3 'no target selected' is shown", "no target selected" in app5._v_status.get(),
        repr(app5._v_status.get()))
    chk("3.3 warning is red", app5._status_lbl.cget("fg") == S._DWELL_LOW_COL)
    # ...and clears once an aircraft turns up and seeds a target
    add_ac(app5, [(51.6, -0.4), (51.7, -0.3)], speed_kt=0)
    frames(app5, clock, 20, draw=False)
    chk("3.3 a target got auto-selected", bool(app5._target_addr_snap),
        str(app5._target_menu_map))
    chk("3.3 warning clears with a target selected", app5._v_status.get() == "",
        repr(app5._v_status.get()))
    # and the warning is not simply never set: prove it toggles back
    app5._v_mode.set("Mode 3/A")
    frames(app5, clock, 4, draw=False)
    chk("3.3 no warning in a non-selective mode", app5._v_status.get() == "")
finally:
    clock.restore()

sys.exit(suite.report())
