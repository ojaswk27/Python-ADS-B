"""Phase 4 — emitter / channel / receiver.

Run directly, or via run_all.py.
"""

import sys
import math

from harness import (Clock, Suite, add_ac, frames, iff, make_app, match_prf,
                     source)
import simulator as S
import channel
import receiver

suite = Suite("Phase 4 — emitter / channel / receiver")
chk = suite.chk



src = source("simulator.py")

# 4.2 aircraft emit bytes only
ac = S.SimAircraft()
ac.waypoints = [(51.6, -0.4), (51.7, -0.3)]
ac.step(1.0)
f = ac.iff_reply(iff.MODE_3A, 42)
chk("4.2 iff_reply returns bytes", isinstance(f, bytes), type(f).__name__)
chk("4.2 iff_reply decodes to the right squawk",
    iff.decode_target_reply(f)["squawk"] == ac.mode3a)
ac.xpdr3a = False
chk("4.2 iff_reply is None with the transponder off",
    ac.iff_reply(iff.MODE_3A, 42) is None)
ac.xpdr3a = True
ac.adsb_frame(0.0)                      # seeds the per-type due times
_af = next((ac.adsb_frame(t) for t in (6.0, 6.6, 7.2, 7.8)
            if ac.adsb_frame(t) is not None), None)
chk("4.2 adsb_frame returns (label, frame)", isinstance(_af, tuple), repr(_af))
chk("4.2 SimAircraft has no colour (display owns that now)",
    "color" not in S.SimAircraft.__slots__)

# The receive path must not touch ground truth.
recv = src.split("# ── ADS-B reception")[0]
recv = recv.split("def _one_interrogation")[1] if "def _one_interrogation" in recv else ""
for leak in ("ac.mode3a", "ac.mode1", "ac.mode2", "ac.callsign", "ac.alt_ft"):
    chk(f"4.2 interrogation path does not read {leak}", leak not in recv)
draw = src.split("# Layer 4: plots")[1].split("# Stale track cleanup")[0]
for leak in ("ac.lat", "ac.lon", "ac.callsign", "ac.alt_ft"):
    ok = leak not in draw.split("if self._truth_on:")[0]
    chk(f"4.5 plot layer does not read {leak}", ok)

# 4.3 channel
site = channel.RadarSite(lat=51.477, lon=-0.461)
chk("4.3 radio horizon grows with altitude",
    channel.radio_horizon_nm(100, 35000) > channel.radio_horizon_nm(100, 1000))
chk("4.3 radio horizon at FL350 is ~242 nm",
    236 < channel.radio_horizon_nm(100, 35000) < 248,
    f"{channel.radio_horizon_nm(100, 35000):.0f} nm")
low = S.SimAircraft(alt_ft=500)
low.waypoints = [(53.5, -0.461)]          # ~121 nm north, below the horizon
low.step(0.0)
rng, ok = channel.visible(low, site, site.iff_max_range_nm)
chk("4.3 a low aircraft beyond the horizon is not visible", not ok,
    f"{rng:.0f} nm vs horizon {channel.radio_horizon_nm(100, 500):.0f} nm")
high = S.SimAircraft(alt_ft=35000)
high.waypoints = [(53.5, -0.461)]
high.step(0.0)
chk("4.3 the same range at FL350 IS visible",
    channel.visible(high, site, site.iff_max_range_nm)[1])
far = S.SimAircraft(alt_ft=35000)
far.waypoints = [(56.0, -0.461)]          # ~271 nm, past the 250 nm limit
far.step(0.0)
chk("4.3 beyond max range is not visible",
    not channel.visible(far, site, site.iff_max_range_nm)[1])

# garble: replies inside the window corrupt each other, both of them
chk("4.3 no garble for a single reply", channel.garbled([10.0]) == set())
chk("4.3 two replies 0.5 nm apart both garble",
    channel.garbled([10.0, 10.5]) == {0, 1})
chk("4.3 two replies 5 nm apart do not garble",
    channel.garbled([10.0, 15.0]) == set())
chk("4.3 garble picks only the close pair from three",
    channel.garbled([10.0, 10.5, 40.0]) == {0, 1},
    str(channel.garbled([10.0, 10.5, 40.0])))
chk("4.3 garble window is ~1.7 nm", abs(channel.GARBLE_WINDOW_NM - 1.7) < 1e-9)

# round-trip delay is the only range source
t_tx = 1000.0
out = channel.deliver_iff(high, site, iff.MODE_3A, t_tx, 7, rng_nm=100.0)
if out is None:                            # 3% reply loss
    out = channel.deliver_iff(high, site, iff.MODE_3A, t_tx, 7, rng_nm=100.0)
frame, t_rx = out
rtt_us = (t_rx - t_tx) * 1e6
chk("4.3 rtt matches 12.3559 us per nm plus turnaround",
    abs(rtt_us - (100 * iff.RADAR_MILE_US + iff.TURNAROUND_AC_US)) < 1e-6,
    f"{rtt_us:.3f} us")
out_s = channel.deliver_iff(high, site, iff.MODE_S_AC, t_tx, 7, rng_nm=100.0)
if out_s:
    rtt_s = (out_s[1] - t_tx) * 1e6
    chk("4.3 Mode S uses the 128 us turnaround",
        abs(rtt_s - (100 * iff.RADAR_MILE_US + iff.TURNAROUND_S_US)) < 1e-6,
        f"{rtt_s:.3f} us")

# 4.4 receiver
rx = receiver.Receiver(site)
trk = rx.rx_iff(frame, t_tx, t_rx, 33.0)
chk("4.4 rx_iff creates a track", trk is not None)
chk("4.4 measured range recovers the true range",
    abs(trk["plot"]["rng_nm"] - 100.0) <= iff.RANGE_QUANT_NM,
    f"{trk['plot']['rng_nm']:.4f} nm")
chk("4.4 range is quantised to the 1/64 nm cell",
    abs(trk["plot"]["rng_nm"] / iff.RANGE_QUANT_NM
        - round(trk["plot"]["rng_nm"] / iff.RANGE_QUANT_NM)) < 1e-9)
chk("4.4 bearing comes from the beam", abs(trk["plot"]["brg_deg"] - 33.0) < 1e-9)
chk("4.4 rx_iff rejects a garbled frame", rx.rx_iff(b"\x00\x01", t_tx, t_rx, 0) is None)

# CPR: one position frame is NOT enough
rx2 = receiver.Receiver(site)
from aircraft_emulator import build_position, build_identification
icao = "4CA1FE"
lat0, lon0 = 51.60, -0.40
t = 500.0
rx2.rx_adsb(build_identification(icao, "BAW117"), t)
tr = rx2.tracks[int(icao, 16)]
chk("4.4 ident alone gives a callsign but no position",
    tr["adsb"].get("call") is not None and tr["adsb"].get("lat") is None)
rx2.rx_adsb(build_position(icao, lat0, lon0, 35000, False), t + 0.1)
chk("4.4 one position frame still gives no position",
    tr["adsb"].get("lat") is None, str(tr["adsb"].get("lat")))
chk("4.4 ...but altitude is already known",
    tr["adsb"].get("alt_ft") is not None)
rx2.rx_adsb(build_position(icao, lat0, lon0, 35000, True), t + 0.2)
chk("4.4 an even/odd pair resolves the position",
    tr["adsb"].get("lat") is not None)
if tr["adsb"].get("lat"):
    err = math.hypot(tr["adsb"]["lat"][0] - lat0,
                     (tr["adsb"]["lon"][0] - lon0) * math.cos(math.radians(lat0))) * 60
    chk("4.4 CPR global decode is accurate", err < 0.02, f"{err * 1852:.0f} m")
# thereafter single frames decode locally
lat1, lon1 = 51.62, -0.38
rx2.rx_adsb(build_position(icao, lat1, lon1, 35000, False), t + 1.0)
err = math.hypot(tr["adsb"]["lat"][0] - lat1,
                 (tr["adsb"]["lon"][0] - lon1) * math.cos(math.radians(lat1))) * 60
chk("4.4 subsequent single frames decode locally", err < 0.02, f"{err * 1852:.0f} m")

# stale pair cannot resolve
rx3 = receiver.Receiver(site)
rx3.rx_adsb(build_position(icao, lat0, lon0, 35000, False), 0.0)
rx3.rx_adsb(build_position(icao, lat0, lon0, 35000, True), 20.0)   # > 10 s apart
chk("4.4 an even/odd pair more than 10 s apart does not resolve",
    rx3.tracks[int(icao, 16)]["adsb"].get("lat") is None)

# 4.5 plots come from the track store
clock = Clock(dt=0.05)
clock.install()
try:
    app = make_app(clock)
    a = add_ac(app, [(51.6, -0.4), (51.7, -0.3)], speed_kt=200)
    app._v_mode.set("Mode 3/A")
    frames(app, clock, 200)
    n_before = len(app.cv.find_withtag("blip"))
    chk("4.5 a tracked aircraft is plotted", n_before > 0, f"{n_before} canvas items")

    # kill every transponder AND ADS-B: the plot must decay away
    a.xpdr1 = a.xpdr2 = a.xpdr3a = a.xpdrC = a.xpdrS = False
    for k in ('POS', 'VEL', 'IDENT'):   # suppress ADS-B emission
        a._adsb_due[k] = 1e18
    clock.advance(6.0)
    frames(app, clock, 6)
    chk("4.5 silent aircraft has no track", not app._tracks, str(list(app._tracks)))
    chk("4.5 silent aircraft is not plotted",
        len(app.cv.find_withtag("blip")) == 0,
        f"{len(app.cv.find_withtag('blip'))} items")
    chk("4.5 ...even though it is still flying",
        a.lat is not None and a.speed_kt > 0)

    # truth overlay is opt-in and draws something when on
    app._v_truth.set(True)
    app._toggle_truth()
    frames(app, clock, 2)
    chk("4.5 truth overlay draws when enabled",
        len(app.cv.find_withtag("blip")) > 0)
    app._v_truth.set(False)
    app._toggle_truth()
    frames(app, clock, 2)
    chk("4.5 truth overlay is off by default again",
        len(app.cv.find_withtag("blip")) == 0)

    # IFF-only plots step, they do not glide: consecutive plot updates are
    # separated by roughly one scan period, not one frame.
    app2 = make_app(clock)
    b = add_ac(app2, [(51.9, -0.461), (52.0, -0.461)], speed_kt=400)
    app2._v_mode.set("Mode 3/A")
    b.xpdrS = False
    seen = []
    for _ in range(400):
        clock.advance(clock.dt)
        app2._frame()
        trk = app2._tracks.get(b.modes_addr)
        if trk and trk["plot"]:
            ts = trk["plot"]["ts"]
            if not seen or ts != seen[-1]:
                seen.append(ts)
    gaps = [round(seen[i + 1] - seen[i], 2) for i in range(len(seen) - 1)]
    scan_s = 60.0 / 15
    big = [g for g in gaps if g > scan_s * 0.5]
    chk("4.5 IFF plot updates are ~one scan apart, not per frame",
        len(big) >= 2 and min(big) > 1.0,
        f"gaps {gaps[:6]} (scan={scan_s:.0f}s)")
    # 4.4 monopulse: azimuth is measured, not the beam pointing angle
    # Reporting the beam angle leaves an error up to half a beamwidth (1.5 deg
    # at the default 3 deg beam), which is ~300 m of cross-range at 25 nm and
    # shifts the IFF plot away from the ADS-B position.  Monopulse estimates the
    # off-boresight angle within a single reply, so the error is the estimator's
    # ~0.1 deg, not the beam's 1.5.
    import statistics as _st
    appM = make_app(clock)
    m = add_ac(appM, [(51.75, -0.15), (52.6, 0.9)], speed_kt=300)
    appM._v_mode.set("Mode 3/A")
    errs = []
    last = None
    for _ in range(3000):
        clock.advance(clock.dt)
        appM._frame()
        t = appM._tracks.get(m.modes_addr)
        if not t or not t["plot"] or t["plot"]["ts"] == last:
            continue
        last = t["plot"]["ts"]
        tb, _tr = iff.bearing_range_nm(appM.c_lat, appM.c_lon, m.lat, m.lon)
        errs.append(abs(iff.angle_diff(t["plot"]["brg_deg"], tb)))
    sd = _st.pstdev(errs)
    chk("4.4 monopulse bearing error is estimator-sized, not beam-sized",
        sd < 0.25, f"sd {sd:.3f} deg (beam half-width is "
                   f"{appM._bw_snap / 2:.1f}, estimator sigma "
                   f"{appM.site.brg_noise_deg})")
    chk("4.4 no bearing error approaches the beam edge",
        max(errs) < appM._bw_snap / 2, f"max {max(errs):.3f} deg")
    # ...and the pre-monopulse mode is beam-limited, which is the contrast
    appM.site.monopulse = False
    errs2 = []
    last = None
    for _ in range(3000):
        clock.advance(clock.dt)
        appM._frame()
        t = appM._tracks.get(m.modes_addr)
        if not t or not t["plot"] or t["plot"]["ts"] == last:
            continue
        last = t["plot"]["ts"]
        tb, _tr = iff.bearing_range_nm(appM.c_lat, appM.c_lon, m.lat, m.lon)
        errs2.append(abs(iff.angle_diff(t["plot"]["brg_deg"], tb)))
    chk("4.4 sliding-window mode is beam-limited by contrast",
        _st.pstdev(errs2) > sd * 1.5,
        f"sd {_st.pstdev(errs2):.3f} vs monopulse {sd:.3f}")

    # 4.5 the plot must not alternate between sources
    # Picking whichever source was freshest made the blip jump to the IFF plot
    # once per scan and straight back, because a reported position is good to
    # metres and a measured plot to hundreds of them.  Detect it as a step that
    # opposes the aircraft's own course: the blip going backwards.
    appJ = make_app(clock)
    j = add_ac(appJ, [(51.75, -0.15), (52.05, 0.25)], speed_kt=300)
    appJ._v_mode.set("Mode 3/A")
    prev = None
    backwards = []
    srcs = {}
    for _ in range(1600):
        clock.advance(clock.dt)
        appJ._frame()
        t = appJ._tracks.get(j.modes_addr)
        if not t:
            continue
        rp = appJ._reported_pos(t, clock.t)
        if rp is None:
            continue
        srcs[rp[2]] = srcs.get(rp[2], 0) + 1
        if prev is not None:
            b, d = iff.bearing_range_nm(prev[0], prev[1], rp[0], rp[1])
            if d * 1852 > 20 and abs(iff.angle_diff(b, j.course())) > 90:
                backwards.append(d * 1852)
        prev = (rp[0], rp[1])
    chk("4.5 the plot never steps backwards along its own course",
        not backwards,
        f"{len(backwards)} backwards steps"
        + (f", max {max(backwards):.0f} m" if backwards else ""))
    chk("4.5 a reported position is preferred over a measured plot",
        srcs.get("adsb", 0) > srcs.get("iff", 0) * 10, str(srcs))
    chk("4.5 ...but the measured plot is still used before CPR resolves",
        srcs.get("iff", 0) > 0, str(srcs))

    # radar site can be moved and resized at runtime
    appS = make_app(clock)
    chk("site fields populate from the live radar",
        appS._v_clat.get().startswith("51.477"), appS._v_clat.get())
    s_ac = add_ac(appS, [(51.6, -0.4), (51.9, -0.1)], speed_kt=300)
    match_prf(appS)
    appS._v_mode.set("Mode 3/A")
    frames(appS, clock, 200, draw=False)
    chk("a track exists before relocating", bool(appS._tracks))
    appS._v_clat.set("53.500")
    appS._apply_site()
    chk("centre moves", appS.c_lat == 53.5, str(appS.c_lat))
    chk("the channel site moves with it", appS.site.lat == 53.5, str(appS.site.lat))
    chk("relocating clears tracks measured from the old site",
        not appS._tracks, str(list(appS._tracks)))
    frames(appS, clock, 300, draw=False)
    t = appS._tracks.get(s_ac.modes_addr)
    if t and t["plot"]:
        tb, tr = iff.bearing_range_nm(appS.c_lat, appS.c_lon, s_ac.lat, s_ac.lon)
        # Bound by travel since the plot, not by an arbitrary tolerance.
        age = clock.t - t["plot"]["ts"]
        budget = s_ac.speed_kt / 3600.0 * age + 2 * iff.RANGE_QUANT_NM
        chk("re-acquires against the new centre",
            abs(t["plot"]["rng_nm"] - tr) <= budget,
            f"{t['plot']['rng_nm']:.2f} vs {tr:.2f} nm, budget {budget:.2f} "
            f"({age:.1f}s old)")
    else:
        chk("re-acquires against the new centre", False, "no plot")
    appS._v_crng.set("120")
    appS._apply_site()
    chk("range applies", appS.rng == 120.0, str(appS.rng))
    appS._v_cant.set("500")
    appS._apply_site()
    chk("antenna height applies", appS.site.ant_height_ft == 500.0,
        str(appS.site.ant_height_ft))
    appS._v_clat.set("999")
    appS._apply_site()
    chk("an out-of-range latitude is refused", appS.c_lat == 53.5, str(appS.c_lat))
    chk("...with a message naming the field",
        "centre lat" in appS._v_siteerr.get(), appS._v_siteerr.get())
    chk("...and the field reverts to the live value",
        appS._v_clat.get() == "53.50000", appS._v_clat.get())
    appS._v_clon.set("abc")
    appS._apply_site()
    chk("a non-numeric value is refused",
        "not a number" in appS._v_siteerr.get(), appS._v_siteerr.get())
    appS._select(s_ac)
    appS._centre_on_selected()
    chk("centre-on-selected moves the radar under the aircraft",
        abs(appS.c_lat - s_ac.lat) < 1e-4,
        f"{appS.c_lat:.5f} vs {s_ac.lat:.5f}")

finally:
    clock.restore()

sys.exit(suite.report())
