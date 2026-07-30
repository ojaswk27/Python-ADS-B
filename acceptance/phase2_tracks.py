"""Phase 2 — one identity, one track store.

Run directly, or via run_all.py.
"""

import sys

from harness import (Clock, Suite, add_ac, frames, make_app, match_prf,
                     source)
import simulator as S

suite = Suite("Phase 2 — one identity, one track store")
chk = suite.chk

src = source("simulator.py")

# ── 2.1 modes_addr is the single source of truth ─────────────────────────────
chk("2.1 icao is derived, not stored", '"icao"' not in src.split("class SimAircraft")[1].split("def __init__")[0])
ac = S.SimAircraft()
chk("2.1 icao derives from modes_addr", ac.icao == f"{ac.modes_addr:06X}", ac.icao)
chk("2.1 _new_id avoids illegal addrs",
    all(S._new_id()[0] not in (0x000000, 0xFFFFFF) for _ in range(500)))
chk("2.1 _new_id uses an allocated block",
    0x400000 <= S.SimAircraft().modes_addr <= 0x43FFFF)
try:
    ac.icao = "ABCDEF"
    chk("2.1 icao is read-only", False)
except AttributeError:
    chk("2.1 icao is read-only", True)

clock = Clock(dt=0.05)
clock.install()
try:
    app = make_app(clock)
    a = add_ac(app, [(51.6, -0.4), (51.7, -0.3)], speed_kt=300)

    # both address entries are the SAME variable
    chk("2.1 MS addr shares the address variable", "_v_msa" not in src)
    chk("2.1 address entry reflects aircraft", app._v_icao.get() == a.icao)

    # rename via the address field updates both readouts and the selection
    app._v_icao.set("4CA1FE")
    app._apply_id()
    chk("2.1 rename applied", a.modes_addr == 0x4CA1FE, hex(a.modes_addr))
    chk("2.1 rename updates address readout", app._v_icao.get() == "4CA1FE",
        app._v_icao.get())
    chk("2.1 rename updates name label", app._v_name.get().startswith("4CA1FE"),
        app._v_name.get())
    chk("2.1 rename repoints _selected_addr", app._selected_addr == 0x4CA1FE,
        hex(app._selected_addr or 0))

    # collision with a live aircraft is rejected
    b = add_ac(app, [(51.5, -0.5), (51.55, -0.45)], speed_kt=300)
    app._select(b)
    before = b.modes_addr
    app._v_icao.set("4CA1FE")
    app._apply_id()
    chk("2.1 colliding rename rejected", b.modes_addr == before, hex(b.modes_addr))
    chk("2.1 rejected rename restores field", app._v_icao.get() == b.icao,
        app._v_icao.get())

    # ── 2.2 one row, one TRK number for an aircraft on both sources ───────────
    app2 = make_app(clock)
    c = add_ac(app2, [(51.6, -0.4), (51.7, -0.3)], speed_kt=300)
    # Field-ageing is what is under test here, not detection probability.  At
    # the defaults the beam yields ~1.1 interrogations per dwell, so a target
    # legitimately gets skipped on some scans (see 3.2 — a real SSR constraint,
    # deliberately not compensated for).  Matching PRF to rotation rate, as an
    # operator would, gives 3.3 hits/dwell and reliable once-per-scan replies.
    match_prf(app2)
    app2._v_mode.set("Mode S All-Call")
    frames(app2, clock, 120)
    chk("2.2 single store keyed by int addr", list(app2._tracks) == [c.modes_addr],
        str([hex(k) for k in app2._tracks]))
    trk = app2._tracks[c.modes_addr]
    chk("2.2 track has iff+adsb+plot subrecords",
        set(trk) >= {"addr", "trk_no", "color", "iff", "adsb", "plot"}, str(sorted(trk)))
    chk("2.2 IFF populated", trk["iff"].get("last_ts") is not None)
    chk("2.2 ADS-B populated", trk["adsb"].get("last_ts") is not None)
    chk("2.2 exactly one table row", len(app2._row_addrs) == 1, str(app2._row_addrs))
    chk("2.2 one counter, no collision", app2.rx.next_track_no == 2,
        f"next={app2.rx.next_track_no}")
    chk("2.2 trk_no is 1", trk["trk_no"] == 1)

    # fields are (value, timestamp) tuples
    app2._v_mode.set("Mode 3/A")
    frames(app2, clock, 120)          # > one 4 s scan, so 3/A is certain
    sq = trk["iff"].get("sqwk")
    chk("2.2 fields are (value, ts) tuples",
        isinstance(sq, tuple) and len(sq) == 2 and sq[0] == c.mode3a, str(sq))
    _sa = S._fld_age(trk["iff"], "sqwk", clock.t)
    chk("2.2 SQWK fresh while 3/A is the active mode",
        S._fld(trk["iff"], "sqwk", clock.t) is not None,
        "never acquired" if _sa is None else f"age {_sa:.1f}s")
    chk("2.2 M1 absent before Mode 1 is ever selected",
        trk["iff"].get("m1") is None)

    # ── 2.2 switching 3/A -> Mode 1 ages SQWK out while M1 stays fresh ────────
    # Stated as "M1 keeps being refreshed while SQWK does not", which is
    # phase-independent.  Asserting "m1 age <= 4 s" at an arbitrary stop time
    # would be a coin flip, since a reply only arrives once per 4 s scan.
    app2._v_mode.set("Mode 1")
    sq_ts_at_switch = trk["iff"]["sqwk"][1]
    frames(app2, clock, 120)                     # 6 s: at least one M1 scan
    m1_ts_first = (trk["iff"].get("m1") or (None, None))[1]
    chk("2.2 M1 acquired after the mode switch", m1_ts_first is not None)
    frames(app2, clock, 120)                     # 6 s more
    m1_ts_later = (trk["iff"].get("m1") or (None, None))[1]
    now = clock.t
    chk("2.2 M1 keeps being refreshed",
        m1_ts_later is not None and m1_ts_later > m1_ts_first,
        f"{m1_ts_first} -> {m1_ts_later}")
    chk("2.2 SQWK is never refreshed again",
        trk["iff"]["sqwk"][1] == sq_ts_at_switch)
    sq_age = now - trk["iff"]["sqwk"][1]
    chk("2.2 SQWK has aged past the stale limit", sq_age > S._FIELD_STALE_S,
        f"sqwk age {sq_age:.1f}s > {S._FIELD_STALE_S}s")
    chk("2.2 SQWK reads as stale", S._fld(trk["iff"], "sqwk", now) is None)
    chk("2.2 SQWK value retained, just stale (not deleted)",
        trk["iff"]["sqwk"][0] == c.mode3a)

    # Render the table at a moment when M1 is known-fresh, so the column check
    # is about formatting rather than about scan phase.
    while S._fld(trk["iff"], "m1", clock.t) is None:
        frames(app2, clock, 10)
    app2._flush_table()
    row = app2._tbl.get("1.0", "1.end")
    probe = app2._table_row(trk="", call="", addr="", sqwk="QQQQ", m1="WW", m2="",
                            alt="", rng="", brg="", age="")
    sq_lo, m1_lo = probe.index("QQQQ"), probe.index("WW")
    chk("2.2 stale SQWK dashes in its own column",
        row[sq_lo:sq_lo + 4].strip() == "—", repr(row[sq_lo:sq_lo + 4]))
    chk("2.2 fresh M1 shown in its own column",
        row[m1_lo:m1_lo + 2].strip() == f"{c.mode1:02o}",
        f"{row[m1_lo:m1_lo + 2]!r} want {c.mode1:02o}")

    # ── 2.3 target menu seeded from ADS-B, tagged by provenance ──────────────
    app3 = make_app(clock)
    d = add_ac(app3, [(51.6, -0.4), (51.7, -0.3)], speed_kt=300)
    # Provenance is under test, not detection probability — match PRF to
    # rotation rate so a reply lands every scan (see 3.2).
    match_prf(app3)
    app3._v_mode.set("Mode 3/A")     # never reveals an address
    frames(app3, clock, 120)
    chk("2.3 ADS-B seeds the address list", d.modes_addr in app3._known_addrs,
        str(app3._known_addrs))
    chk("2.3 tagged as ADS-B-derived", app3._known_addrs[d.modes_addr] == "A",
        str(app3._known_addrs))
    labels = list(app3._target_menu_map)
    chk("2.3 dropdown shows the A tag", labels and labels[0].endswith(" A"), str(labels))
    chk("2.3 dropdown maps to the int addr",
        app3._target_menu_map.get(labels[0]) == d.modes_addr)
    # a Mode S all-call upgrades the provenance to S
    app3._v_mode.set("Mode S All-Call")
    frames(app3, clock, 120)
    chk("2.3 Mode S upgrades provenance to S",
        app3._known_addrs[d.modes_addr] == "S", str(app3._known_addrs))
    labels = list(app3._target_menu_map)
    chk("2.3 dropdown shows the S tag", labels and labels[0].endswith(" S"), str(labels))
    # ...and never downgrades back to A
    frames(app3, clock, 120)
    chk("2.3 provenance never downgrades", app3._known_addrs[d.modes_addr] == "S")

    # ── table must not recompute truth geometry ──────────────────────────────
    body = src.split("def _flush_table")[1].split("def _refresh_detail")[0]
    chk("2.2 _flush_table has no ground-truth fallback",
        "last_lat" not in body and "ac.lat" not in body)
finally:
    clock.restore()

sys.exit(suite.report())
