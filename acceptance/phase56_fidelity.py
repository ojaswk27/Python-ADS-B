"""Phase 5 & 6 — protocol fidelity, cleanup.

Run directly, or via run_all.py.
"""

import sys

from harness import (Clock, Suite, add_ac, frames, iff, make_app,
                     match_prf, source)
import simulator as S

suite = Suite("Phase 5 & 6 — protocol fidelity, cleanup")
chk = suite.chk

from aircraft_emulator import decode_frame


src = source("simulator.py")

# 5.1 Mode C quantisation and QNH
chk("5.1 Mode C quantises to 100 ft",
    iff.encode_mode_c_alt(35049) == 35000 and iff.encode_mode_c_alt(35051) == 35100,
    f"{iff.encode_mode_c_alt(35049)} / {iff.encode_mode_c_alt(35051)}")
chk("5.1 standard QNH is a no-op at round altitudes",
    iff.encode_mode_c_alt(35000) == 35000)
# 1013.25 -> 993.25 is 20 hPa low, so pressure altitude reads ~540 ft HIGH
chk("5.1 low QNH raises reported pressure altitude",
    iff.encode_mode_c_alt(35000, 993.25) == 35500,
    str(iff.encode_mode_c_alt(35000, 993.25)))
chk("5.1 high QNH lowers it",
    iff.encode_mode_c_alt(35000, 1033.25) == 34500,
    str(iff.encode_mode_c_alt(35000, 1033.25)))
chk("5.1 clamped to the Mode C floor", iff.encode_mode_c_alt(-5000) == -1000)
chk("5.1 clamped to the Mode C ceiling", iff.encode_mode_c_alt(200000) == 126700)

# the reply actually carries pressure altitude, not geometric
ac = S.SimAircraft(alt_ft=35049)
ac.waypoints = [(51.6, -0.4), (51.7, -0.3)]
ac.step(0.0)
got = iff.decode_target_reply(ac.iff_reply(iff.MODE_C, 1))["alt_ft"]
chk("5.1 Mode C reply is quantised, not 1 ft resolution",
    got % 25 == 0 and abs(got - 35000) <= 25, f"{got} ft from 35049 ft geometric")
got_q = iff.decode_target_reply(ac.iff_reply(iff.MODE_C, 1, 993.25))["alt_ft"]
chk("5.1 QNH reaches the reply", got_q != got, f"{got} vs {got_q}")

# 5.2 Mode 1 code range
chk("5.2 0o73 is the max legal Mode 1 code",
    iff.valid_mode1(0o73) and not iff.valid_mode1(0o74))
chk("5.2 second digit is limited to 0-3",
    iff.valid_mode1(0o63) and not iff.valid_mode1(0o64))
chk("5.2 first digit runs 0-7", iff.valid_mode1(0o70) and iff.valid_mode1(0o03))
chk("5.2 exactly 32 legal codes",
    sum(1 for v in range(0o100) if iff.valid_mode1(v)) == 32)
chk("5.2 random init is always legal",
    all(iff.valid_mode1(S.SimAircraft().mode1) for _ in range(200)))

# 5.3 ADS-B message rates and mix
clock = Clock(dt=0.05)
clock.install()
try:
    a = S.SimAircraft(alt_ft=35000, speed_kt=450)
    a.waypoints = [(51.0, -0.4), (52.0, -0.4)]
    a.step(0.0)
    counts = {}
    t = 0.0
    for _ in range(12000):            # 60 s at dt=0.005
        t += 0.005
        out = a.adsb_frame(t)
        if out:
            counts[out[0]] = counts.get(out[0], 0) + 1
    pos = counts.get("POS-E", 0) + counts.get("POS-O", 0)
    vel = counts.get("VEL", 0)
    idt = counts.get("IDENT", 0)
    total = pos + vel + idt
    chk("5.3 position rate is ~2 Hz", 1.7 <= pos / 60 <= 2.3, f"{pos / 60:.2f} Hz")
    chk("5.3 velocity rate is ~2 Hz", 1.7 <= vel / 60 <= 2.3, f"{vel / 60:.2f} Hz")
    chk("5.3 ident rate is ~0.2 Hz", 0.15 <= idt / 60 <= 0.25, f"{idt / 60:.2f} Hz")
    chk("5.3 total is ~4.2 msg/s, not the old 12.5",
        3.5 <= total / 60 <= 5.0, f"{total / 60:.2f} msg/s")
    chk("5.3 position alternates even/odd",
        abs(counts.get("POS-E", 0) - counts.get("POS-O", 0)) <= 1,
        f"E={counts.get('POS-E')} O={counts.get('POS-O')}")
    chk("5.3 intervals are randomised, not fixed",
        "random.uniform" in src)

    # 5.4 velocity carries a real vertical rate
    b = S.SimAircraft(alt_ft=10000, speed_kt=450)
    b.waypoints = [(51.0, -0.4), (52.0, -0.4)]
    b.step(0.0)
    chk("5.4 level flight reports zero vertical rate", b.vrate_fpm == 0.0)
    b.alt_target_ft = 30000.0
    b.step(1.0)
    chk("5.4 climbing reports a positive vertical rate",
        b.vrate_fpm == S._CLIMB_FPM, str(b.vrate_fpm))
    chk("5.4 altitude actually moves toward the target",
        9999 < b.alt_ft < 10031, f"{b.alt_ft:.1f} ft after 1 s at 1800 fpm")
    d = decode_frame(b.adsb_frame(1000.0)[1]) if b.adsb_frame(1000.0) else None
    b._adsb_due.clear(); b.adsb_frame(0.0)
    vf = None
    for tt in (0.6, 1.2, 1.8, 2.4, 3.0):
        o = b.adsb_frame(tt)
        if o and o[0] == "VEL":
            vf = decode_frame(o[1]); break
    if vf:
        chk("5.4 vertical rate reaches the ADS-B velocity frame",
            vf["vrate_fpm"] is not None and abs(vf["vrate_fpm"] - 1800) <= 64,
            str(vf["vrate_fpm"]))
    b.alt_target_ft = 5000.0
    b.step(1.0)
    chk("5.4 descending reports a negative vertical rate", b.vrate_fpm < 0,
        str(b.vrate_fpm))
    b.alt_target_ft = b.alt_ft
    b.step(1.0)
    chk("5.4 levelling off returns to zero", b.vrate_fpm == 0.0)
    chk("5.4 velocity field is named track, not heading",
        "track_deg" in source("receiver.py"))

    # 5.5 turn rate is opt-in; instant by default
    c = S.SimAircraft(speed_kt=3600)
    c.waypoints = [(51.0, -0.4), (52.0, -0.4), (52.0, 1.0)]   # 90 deg corner
    chk("5.5 default turn rate is instant", c.turn_rate_deg_s == 0.0,
        str(c.turn_rate_deg_s))
    seen = set()
    for _ in range(4000):
        c.step(0.05)
        seen.add(round(c.heading(), 3))
    chk("5.5 instant mode reports the course as flown, jump included",
        abs(iff.angle_diff(c.heading(), c.course())) < 1e-9)
    chk("5.5 instant mode steps rather than sweeping (few distinct headings)",
        len(seen) <= 4, f"{len(seen)} distinct headings over the corner")

    # ...and the limiter still works when asked for
    c2 = S.SimAircraft(speed_kt=3600)
    c2.waypoints = [(51.0, -0.4), (52.0, -0.4), (52.0, 1.0)]
    c2.turn_rate_deg_s = 3.0
    worst = 0.0
    prev = None
    swept = 0
    for _ in range(4000):
        c2.step(0.05)
        h = c2.heading()
        if prev is not None:
            rate = abs(iff.angle_diff(h, prev)) / 0.05
            worst = max(worst, rate)
            if 0 < rate:
                swept += 1
        prev = h
    chk("5.5 an explicit turn rate is honoured",
        worst <= 3.0 + 1e-6, f"worst {worst:.3f} deg/s")
    chk("5.5 ...and it actually sweeps rather than snapping", swept > 100,
        f"{swept} frames turning")
    chk("5.5 heading converges once the turn completes",
        abs(iff.angle_diff(c2.heading(), c2.course())) < 1e-6)
    chk("5.5 course still steps instantly regardless",
        abs(iff.angle_diff(c2.course(), 90.0)) < 1.0, f"course {c2.course():.1f}")

    # 5.5 regression: no fabricated northbound heading on acquisition
    # An aircraft created by clicking a single waypoint has a position but no
    # course.  course() falls back to 0.0, so latching that into _hdg (or
    # emitting it in a velocity frame) made every aircraft appear northbound
    # and then swing to its real track at 3 deg/s.
    app9 = make_app(clock)
    match_prf(app9)
    app9._new_ac()
    n9 = app9._selected
    n9.waypoints.append((51.60, -0.40))
    frames(app9, clock, 60, draw=False)
    chk("5.5 no _hdg latched with a single waypoint", n9._hdg is None, str(n9._hdg))
    t9 = app9._tracks.get(n9.modes_addr)
    chk("5.5 single-waypoint aircraft reports no track angle",
        not (t9 and t9["adsb"].get("track_deg")),
        str(t9["adsb"].get("track_deg") if t9 else None))
    n9.waypoints.append((51.55, -0.20))
    want9 = n9.course()
    frames(app9, clock, 1, draw=False)
    chk("5.5 heading is right on the first frame of a new leg",
        abs(iff.angle_diff(n9.heading(), want9)) < 1e-9,
        f"{n9.heading():.2f} vs {want9:.2f}")
    got9 = None
    for _ in range(400):
        clock.advance(clock.dt)
        app9._frame()
        tt = app9._tracks.get(n9.modes_addr)
        if tt and tt["adsb"].get("track_deg"):
            got9 = tt["adsb"]["track_deg"][0]
            break
    chk("5.5 first decoded track matches the course, not north",
        got9 is not None and abs(iff.angle_diff(got9, want9)) < 1.0,
        f"{got9} vs {want9:.2f}")
    # A stopped aircraft likewise reports no track.
    n9.speed_kt = 0
    chk("5.5 a stopped aircraft emits no velocity frame",
        all((o := n9.adsb_frame(clock.t + k)) is None or o[0] != "VEL"
            for k in [x * 0.1 for x in range(1, 60)]))

    # 5.6 one geometry implementation
    chk("5.6 no _dist_nm helper left", "_dist_nm" not in src)
    chk("5.6 no _bearing helper left", "def _bearing" not in src)
    chk("5.6 simulator uses iff.bearing_range_nm", "iff.bearing_range_nm" in src)

    # 5.7 emergency codes and SPI
    chk("5.7 7500 is HIJACK", iff.emergency_label(0o7500) == "HIJACK")
    chk("5.7 7600 is RADIO FAIL", iff.emergency_label(0o7600) == "RADIO FAIL")
    chk("5.7 7700 is EMERGENCY", iff.emergency_label(0o7700) == "EMERGENCY")
    chk("5.7 an ordinary squawk has no label", iff.emergency_label(0o1200) is None)

    app = make_app(clock)
    e = add_ac(app, [(51.6, -0.4), (51.7, -0.3)], speed_kt=200)
    e.mode3a = 0o7700
    # Match PRF to rotation rate so acquisition is reliable: at the defaults the
    # beam gives ~1.1 hits/dwell and a target is skipped on some scans (3.2).
    match_prf(app)
    app._v_mode.set("Mode 3/A")
    frames(app, clock, 200)
    app._flush_table()
    body = app._tbl.get("1.0", tk_end := "end")
    chk("5.7 emergency squawk is called out in the table", "EMERGENCY" in body,
        repr(body[:90]))
    tags = app._tbl.tag_names()
    chk("5.7 emergency row has its own tag", "emerg" in tags, str(tags))

    # SPI / IDENT
    app._select(e)
    app._ident()
    chk("5.7 IDENT arms SPI", e.spi_until > clock.t)
    chk("5.7 SPI lasts 18 s", abs((e.spi_until - clock.t) - S._SPI_S) < 0.2,
        f"{e.spi_until - clock.t:.1f}s")
    e.mode3a = 0o1200
    frames(app, clock, 200)
    app._flush_table()
    body = app._tbl.get("1.0", "end")
    chk("5.7 SPI shows IDENT on the row", "IDENT" in body, repr(body[:90]))
    clock.advance(S._SPI_S + 1)
    frames(app, clock, 200)
    app._flush_table()
    chk("5.7 SPI expires", "IDENT" not in app._tbl.get("1.0", "end"))

    # 6.1 duplicate address alert
    app2 = make_app(clock)
    p = add_ac(app2, [(51.6, -0.4), (51.7, -0.3)], speed_kt=200)
    q = add_ac(app2, [(51.5, -0.5), (51.55, -0.45)], speed_kt=200)
    frames(app2, clock, 20)
    chk("6.1 no alert with distinct addresses",
        "duplicate" not in app2._v_status.get(), repr(app2._v_status.get()))
    q.modes_addr = p.modes_addr
    frames(app2, clock, 20)
    app2._flush_table()
    chk("6.1 duplicate address raises a visible alert",
        "duplicate address" in app2._v_status.get(), repr(app2._v_status.get()))
    chk("6.1 alert names the address",
        f"{p.modes_addr:06X}" in app2._v_status.get(), repr(app2._v_status.get()))

    # 6.2/6.3 table bookkeeping
    ft = src.split("def _flush_table")[1].split("def _refresh_detail")[0]
    chk("6.3 _table_dirty is cleared at the END of _flush_table",
        ft.rstrip().endswith("self._table_dirty = False"),
        repr(ft.rstrip()[-60:]))
    chk("6.2 target menu refreshed inside the same pass",
        "_refresh_target_menu()" in ft)

    # 6.4 Mode 2 is a radar-level unit code
    app3 = make_app(clock)
    app3._v_unit2.set("7654")
    app3._apply_unit2()
    app3._new_ac()
    chk("6.4 new aircraft inherit the unit code",
        app3._aircraft[-1].mode2 == 0o7654, f"{app3._aircraft[-1].mode2:04o}")
    app3._aircraft[-1].mode2 = 0o1111
    app3._new_ac()
    chk("6.4 unit code still applies to the next aircraft",
        app3._aircraft[-1].mode2 == 0o7654)
    chk("6.4 per-aircraft override is preserved",
        app3._aircraft[-2].mode2 == 0o1111)

    # 6.6 regression: the target dropdown must not churn
    # _target_menu_dirty was set on every received frame, so at ~4 ADS-B msg/s
    # per aircraft the menu was deleted and re-added several times a second,
    # tearing the list out from under anyone trying to click an entry.
    appA = make_app(clock)
    match_prf(appA)
    for i in range(4):
        add_ac(appA, [(51.55 + 0.04 * i, -0.45 + 0.03 * i),
                      (51.75 + 0.04 * i, -0.15 + 0.03 * i)], speed_kt=280)
    appA._v_mode.set("Mode S All-Call")
    frames(appA, clock, 200, draw=False)
    chk("6.6 all four aircraft reach the dropdown",
        len(appA._target_menu_map) == 4, f"{len(appA._target_menu_map)}")
    menuA = appA._target_om["menu"]
    rebuilds = [0]
    _od = menuA.delete
    menuA.delete = lambda *a, **k: (rebuilds.__setitem__(0, rebuilds[0] + 1),
                                    _od(*a, **k))[1]
    frames(appA, clock, 400, draw=False)          # 20 s of steady traffic
    chk("6.6 menu does not rebuild during steady traffic", rebuilds[0] == 0,
        f"{rebuilds[0]} rebuilds in 20 s")
    pick = list(appA._target_menu_map)[2]
    appA._v_target.set(pick)
    frames(appA, clock, 400, draw=False)
    chk("6.6 a user selection survives continued reception",
        appA._v_target.get() == pick, f"{appA._v_target.get()!r}")
    chk("6.6 selection still resolves to its address",
        appA._target_addr_snap == appA._target_menu_map[pick])
    add_ac(appA, [(51.9, -0.6), (52.0, -0.5)], speed_kt=280)
    frames(appA, clock, 200, draw=False)
    chk("6.6 a genuinely new aircraft is still added",
        len(appA._target_menu_map) == 5, f"{len(appA._target_menu_map)}")
    chk("6.6 ...without disturbing the selection",
        appA._v_target.get() == pick, appA._v_target.get())

finally:
    clock.restore()

sys.exit(suite.report())
