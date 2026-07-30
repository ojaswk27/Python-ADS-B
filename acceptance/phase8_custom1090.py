"""Custom 1090 format — end to end in the running application.

Run directly, or via run_all.py.
"""

import sys

from harness import (Clock, Suite, add_ac, frames, iff, make_app, match_prf,
                     source)
import pseudo1090 as ps
import simulator as S

suite = Suite("Custom 1090 — replaces ADS-B on the same PHY")
chk = suite.chk

src = source("simulator.py")


def kinds(app):
    out = {}
    for _ts, k, _l, _f, _d, _lost in app._log:
        out[k] = out.get(k, 0) + 1
    return out


clock = Clock(dt=0.05)
clock.install()
try:
    # ── the shipped default must not change existing behaviour ────────────────
    app = make_app(clock)
    chk("default transmit mode is standard ADS-B",
        app._tx_mode == ps.MODE_STANDARD, app._tx_mode)
    chk("a format is loaded and valid", app.fmt is not None)
    chk("no config error on the shipped file", app.fmt_error is None,
        str(app.fmt_error))

    # ── standard mode: only ADS-B on the air ─────────────────────────────────
    a = add_ac(app, [(51.6, -0.4), (51.9, -0.1)], speed_kt=350)
    match_prf(app)
    frames(app, clock, 200, draw=False)
    k = kinds(app)
    chk("standard mode transmits ADS-B", k.get("ADSB", 0) > 0, str(k))
    chk("standard mode transmits no custom frames", "CUSTOM" not in k, str(k))
    trk = app._tracks[a.modes_addr]
    chk("standard mode populates the adsb record",
        trk["adsb"].get("last_ts") is not None)
    chk("standard mode leaves the custom record empty",
        trk["pseudo"].get("last_ts") is None)

    # ── custom mode: ADS-B replaced entirely ─────────────────────────────────
    app2 = make_app(clock)
    app2._v_txmode.set(ps.MODE_CUSTOM)
    app2._apply_txmode()
    b = add_ac(app2, [(51.6, -0.4), (51.9, -0.1)], speed_kt=350)
    match_prf(app2)
    frames(app2, clock, 200, draw=False)
    k = kinds(app2)
    chk("custom mode transmits the custom format", k.get("CUSTOM", 0) > 0, str(k))
    chk("custom mode transmits NO ADS-B", "ADSB" not in k, str(k))
    t2 = app2._tracks[b.modes_addr]
    chk("custom mode populates the custom record",
        t2["pseudo"].get("last_ts") is not None)
    chk("custom mode leaves the adsb record empty",
        t2["adsb"].get("last_ts") is None)
    chk("every custom frame decoded", app2.rx.undecodable == 0,
        f"{app2.rx.undecodable} undecodable")
    chk("every custom frame was attributed", app2.rx.unattributed == 0,
        f"{app2.rx.unattributed} unattributed")
    chk("address learned from the custom format",
        app2._known_addrs.get(b.modes_addr) == "C",
        str(app2._known_addrs))
    chk("dropdown tags the custom-derived address",
        any(lbl.endswith(" C") for lbl in app2._target_menu_map),
        str(list(app2._target_menu_map)))

    # all three message types get through
    msgs = {}
    for _ts, kk, lbl, _f, _d, _lost in app2._log:
        if kk == "CUSTOM":
            msgs[lbl] = msgs.get(lbl, 0) + 1
    chk("all three configured message types transmit",
        set(msgs) == {"position", "ident", "velocity"}, str(msgs))

    # The log label above comes from the EMITTER, so on its own it says nothing
    # about whether the receiver read the discriminator.  Decode the logged
    # frames independently and check the type round-trips: without this, a
    # decoder that ignored type_field entirely would still pass.
    mismatched = []
    for _ts, kk, lbl, frame, _d, lost in app2._log:
        if kk != "CUSTOM" or lost or not frame:
            continue
        got_name, _vals = app2.fmt.decode(frame)
        if got_name != lbl:
            mismatched.append((lbl, got_name))
    chk("each frame decodes back to the type that was sent",
        not mismatched, f"{len(mismatched)} mismatched, e.g. {mismatched[:3]}")
    chk("the receiver recorded the decoded type, not the sent one",
        t2["pseudo"].get("last_type") in ("position", "ident", "velocity"),
        str(t2["pseudo"].get("last_type")))
    # the configured periods are honoured: position ~2 Hz, ident ~0.2 Hz
    secs = 200 * clock.dt
    chk("position honours its 0.5 s period",
        1.6 <= msgs.get("position", 0) / secs <= 2.4,
        f"{msgs.get('position', 0) / secs:.2f} Hz")
    chk("ident honours its 5 s period",
        0.1 <= msgs.get("ident", 0) / secs <= 0.35,
        f"{msgs.get('ident', 0) / secs:.2f} Hz")

    # ── the whole point: a custom-only track still plots ─────────────────────
    app3 = make_app(clock)
    app3._v_txmode.set(ps.MODE_CUSTOM)
    app3._apply_txmode()
    c = add_ac(app3, [(51.6, -0.4), (51.9, -0.1)], speed_kt=350)
    c.xpdr1 = c.xpdr2 = c.xpdr3a = c.xpdrC = c.xpdrS = False   # no IFF at all
    frames(app3, clock, 200)
    t3 = app3._tracks[c.modes_addr]
    rp = app3._reported_pos(t3, clock.t)
    chk("a custom-only track has a reported position", rp is not None)
    if rp:
        chk("...sourced from the custom format", rp[2] == "pseudo", rp[2])
        # The error against live truth is dominated by how old the last frame
        # is, not by the encoding: 24-bit lat over ±90 is 1.2 m, but 350 kt
        # covers 180 m/s and a frame can be a second old after a channel loss.
        # So bound it by distance travelled since the frame, plus resolution.
        age = clock.t - t3["pseudo"]["lat"][1]
        res_m = ps.load(ps.DEFAULT_CFG).messages["position"] \
                  .field("lat").resolution() * 60 * 1852
        budget = c.speed_kt / 3600.0 * 1852 * age + res_m + 5.0
        err_m = iff.bearing_range_nm(rp[0], rp[1], c.lat, c.lon)[1] * 1852
        chk("...as accurate as its age allows", err_m <= budget,
            f"{err_m:.0f} m vs budget {budget:.0f} m "
            f"(frame {age:.2f}s old, encoding {res_m:.1f} m)")
    chk("a custom-only track is plotted",
        len(app3.cv.find_withtag("blip")) > 0,
        f"{len(app3.cv.find_withtag('blip'))} items")
    row = app3._tbl.get("1.0", "1.end")
    chk("a custom-only track appears in the table", "TRK" not in row and row.strip(),
        repr(row))
    chk("the table shows its address", f"{c.modes_addr:06X}" in row, repr(row))
    chk("the table shows its altitude", "FL350" in row, repr(row))
    chk("the decoded pane names the custom source",
        "custom 1090" in app3._detail.get("1.0", "end"))
    chk("the decoded pane lists decoded custom fields",
        "C1090" in app3._detail.get("1.0", "end"))

    # ── both mode: two formats on one channel ────────────────────────────────
    app4 = make_app(clock)
    app4._v_txmode.set(ps.MODE_BOTH)
    app4._apply_txmode()
    d = add_ac(app4, [(51.6, -0.4), (51.9, -0.1)], speed_kt=350)
    match_prf(app4)
    frames(app4, clock, 300, draw=False)
    k = kinds(app4)
    chk("both mode transmits ADS-B", k.get("ADSB", 0) > 0, str(k))
    chk("both mode transmits the custom format", k.get("CUSTOM", 0) > 0, str(k))
    t4 = app4._tracks[d.modes_addr]
    chk("both records populate on one track",
        t4["adsb"].get("last_ts") is not None
        and t4["pseudo"].get("last_ts") is not None)
    chk("both formats share one track number", len(app4._tracks) == 1,
        str(list(app4._tracks)))
    chk("neither stream confuses the other's decoder",
        app4.rx.undecodable == 0, f"{app4.rx.undecodable} undecodable")
    # provenance must be stable, or anything keyed on it churns
    seen = set()
    for _ in range(200):
        clock.advance(clock.dt)
        app4._frame()
        seen.add(app4._known_addrs.get(d.modes_addr))
    chk("provenance is stable with two sources active", len(seen) == 1,
        str(seen))

    # ── the log records airtime, including losses ────────────────────────────
    chk("the log is capped", len(app4._log) <= S._LOG_MAX,
        f"{len(app4._log)} <= {S._LOG_MAX}")
    chk("the log holds the transmitted hex",
        all(e[3] is None or len(e[3]) == 28 for e in app4._log))
    chk("the log pane rendered lines",
        len(app4._log_txt.get("1.0", "end").strip().splitlines()) > 0)
    chk("the log status counts transmissions",
        "tx" in app4._v_logstat.get(), app4._v_logstat.get())

    # frames lost in the channel are recorded as lost, not silently dropped
    app5 = make_app(clock)
    app5._v_txmode.set(ps.MODE_CUSTOM)
    app5._apply_txmode()
    app5.site.adsb_frame_prob = 0.0          # lose everything on 1090
    e = add_ac(app5, [(51.6, -0.4), (51.9, -0.1)], speed_kt=350)
    # IFF is a separate channel and would create a track on its own, which
    # would say nothing about 1090 loss.  Silence it so this measures one thing.
    e.xpdr1 = e.xpdr2 = e.xpdr3a = e.xpdrC = e.xpdrS = False
    frames(app5, clock, 100, draw=False)
    lost = [x for x in app5._log if x[5]]
    chk("channel losses are logged as lost", len(lost) > 0, f"{len(lost)}")
    chk("every transmission was lost", len(lost) == len(app5._log),
        f"{len(lost)}/{len(app5._log)}")
    chk("a fully lossy 1090 channel yields no track", not app5._tracks,
        str(list(app5._tracks)))

    # ── a bad config must not stop the simulator ─────────────────────────────
    import tempfile, os
    bad = os.path.join(tempfile.mkdtemp(), "bad.cfg")
    with open(bad, "w") as fh:
        fh.write("[format]\nmode = custom\n[msg:a]\nx = 80 12 uint:1 ac.alt_ft\n")
    app6 = S.CombinedApp(51.477, -0.461, 200.0, 0.0, cfg_path=bad)
    app6._cw = app6._ch = S.ui.CANVAS_SZ
    app6._tick = clock.t
    app6.update()
    chk("a bad config does not stop startup", app6.fmt is None)
    chk("the error is retained for display", bool(app6.fmt_error))
    chk("a bad config falls back to standard ADS-B",
        app6._tx_mode == ps.MODE_STANDARD, app6._tx_mode)
    app6._v_txmode.set(ps.MODE_CUSTOM)
    app6._apply_txmode()
    chk("custom mode is refused while no format is loaded",
        app6._tx_mode == ps.MODE_STANDARD, app6._tx_mode)
    add_ac(app6, [(51.6, -0.4), (51.9, -0.1)], speed_kt=350)
    frames(app6, clock, 60, draw=False)
    chk("...and it still transmits ADS-B", kinds(app6).get("ADSB", 0) > 0,
        str(kinds(app6)))
    app6.destroy()

    # ── structural: the custom path reuses the shared physical layer ─────────
    tx = src.split("# 4. 1090 MHz")[1].split("# 5. Refresh")[0]
    chk("custom frames go through the same channel as ADS-B",
        tx.count("channel.deliver_adsb") == 2, str(tx.count("channel.deliver_adsb")))
    chk("the receiver, not the emitter, decodes custom frames",
        "rx.rx_pseudo" in tx)
finally:
    clock.restore()

sys.exit(suite.report())
