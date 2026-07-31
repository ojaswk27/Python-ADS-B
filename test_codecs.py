#!/usr/bin/env python3
"""
Codec round-trip tests

Every encoder in the sim must survive a trip through its decoder.  These run
before the receiver is wired up, so a decode bug shows here rather than as a
mystery blank column in the track table.

    python test_codecs.py
"""

import math
import random
import sys

import iff_protocol as iff
from aircraft_emulator import (build_identification, build_position,
                               build_velocity, decode_frame)
import adsb_decoder as dec

_fails = []


def chk(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"   {detail}" if detail else ""))
    if not cond:
        _fails.append(name)


def near(a, b, tol):
    return a is not None and b is not None and abs(a - b) <= tol


# IFF per-target reply

def test_iff_roundtrip():
    print("\niff_protocol.encode_target_reply -> decode_target_reply")
    addr, prt, lat, lon = 0x4CA1FE, 12345, 51.4775, -0.4614

    cases = [
        (iff.MODE_1,  {"mission_code": 0o73},                     "mission_code", 0o73),
        (iff.MODE_1,  {"mission_code": 0},                        "mission_code", 0),
        (iff.MODE_2,  {"unit_code": 0o7777},                      "unit_code",    0o7777),
        (iff.MODE_3A, {"squawk": 0o7700},                          "squawk",       0o7700),
        (iff.MODE_3A, {"squawk": 0},                               "squawk",       0),
        (iff.MODE_S_AC,  {"modes_addr": addr},                     "modes_addr",   addr),
        (iff.MODE_S_SEL, {"modes_addr": addr, "bds_reg": iff.BDS_CALLSIGN,
                          "callsign": "BAW117"},                   "callsign",     "BAW117"),
    ]
    for mode, kw, key, want in cases:
        raw = iff.encode_target_reply(addr, mode, prt, lat, lon, **kw)
        got = iff.decode_target_reply(raw)
        name = iff.MODE_NAMES[mode]
        chk(f"{name} {key} survives", got.get(key) == want,
            f"got {got.get(key)!r} want {want!r}")
        chk(f"{name} header survives",
            got["icao"] == addr and got["mode"] == mode and got["prt_no"] == prt,
            f"{got['icao']:06X} mode={got['mode']} prt={got['prt_no']}")
        chk(f"{name} position survives",
            near(got["src_lat"], lat, 1e-4) and near(got["src_lon"], lon, 1e-4),
            f"{got['src_lat']:.5f},{got['src_lon']:.5f}")
        # A mode must not leak fields it does not carry.
        others = {"mission_code", "unit_code", "squawk", "callsign"} - {key}
        chk(f"{name} leaks no other payload field",
            not (others & set(got)), str(others & set(got)))

    # Mode C is lossy by design (25 ft), so check the quantisation explicitly.
    for alt in (-1000, -25, 0, 25, 5000, 35000, 50000):
        raw = iff.encode_target_reply(addr, iff.MODE_C, prt, lat, lon, alt_ft=alt)
        got = iff.decode_target_reply(raw)["alt_ft"]
        chk(f"MC {alt:>6} ft round-trips within 25 ft", near(got, alt, 25),
            f"got {got}")

    # Every squawk value, exhaustively.
    bad = [v for v in range(0o10000)
           if iff.decode_target_reply(
               iff.encode_target_reply(addr, iff.MODE_3A, prt, lat, lon, squawk=v)
           )["squawk"] != v]
    chk("all 4096 squawk codes round-trip", not bad, f"{len(bad)} bad")

    # Every legal Mode 1 code, in display (octal-digit) form.  Mode 1 is
    # A(3 bits) + B(2 bits): first digit 0-7, second 0-3 — 00 to 73 octal, 32
    # codes, sparse over 0..59.  Masking with 0x1F would fold 0o73 onto 0o33.
    legal = [v for v in range(0o100) if iff.valid_mode1(v)]
    chk("32 legal Mode 1 display codes", len(legal) == 32, str(len(legal)))
    chk("0o73 is legal and 0o74 is not",
        iff.valid_mode1(0o73) and not iff.valid_mode1(0o74))
    bad = [v for v in legal
           if iff.decode_target_reply(
               iff.encode_target_reply(addr, iff.MODE_1, prt, lat, lon, mission_code=v)
           )["mission_code"] != v]
    chk("all 32 Mode 1 codes round-trip", not bad,
        f"{len(bad)} bad: {[oct(v) for v in bad[:5]]}")
    chk("Mode 1 display form maps onto all 32 wire values",
        sorted({iff.mode1_to_wire(v) for v in legal}) == list(range(32)))
    try:
        iff.encode_target_reply(addr, iff.MODE_1, prt, lat, lon, mission_code=0o74)
        chk("illegal Mode 1 code is rejected, not truncated", False)
    except ValueError:
        chk("illegal Mode 1 code is rejected, not truncated", True)

    # Callsign alphabet.
    for cs in ("", "A", "BAW117", "SIM001", "ZZZZZZZZ", "N123AB"):
        raw = iff.encode_target_reply(addr, iff.MODE_S_SEL, prt, lat, lon,
                                      modes_addr=addr, bds_reg=iff.BDS_CALLSIGN,
                                      callsign=cs)
        got = iff.decode_target_reply(raw)["callsign"]
        chk(f"callsign {cs!r} round-trips", got == cs, f"got {got!r}")


def test_iff_rejects_malformed():
    print("\niff_protocol.decode_target_reply rejects malformed frames")
    good = iff.encode_target_reply(0x4CA1FE, iff.MODE_3A, 1, 51.0, -0.4, squawk=0o1200)

    def raises(pkt, label):
        try:
            iff.decode_target_reply(pkt)
        except ValueError:
            chk(f"rejects {label}", True)
        except Exception as e:
            chk(f"rejects {label}", False, f"raised {type(e).__name__}, want ValueError")
        else:
            chk(f"rejects {label}", False, "decoded without error")

    raises(b"", "an empty frame")
    raises(good[:5], "a truncated header")
    raises(good[:-1], "a truncated payload")
    raises(good + b"\x00", "an over-long payload")
    raises(bytearray(good[:3]) + b"\x63" + good[4:], "an unknown mode byte")
    chk("accepts the good frame", iff.decode_target_reply(good)["squawk"] == 0o1200)


# ADS-B frames

def test_adsb_ident():
    print("\naircraft_emulator.build_identification -> decode_frame")
    for icao, cs in (("4CA1FE", "BAW117"), ("400001", "SIM001"),
                     ("A05F21", "UAL456"), ("3C6444", "DLH400")):
        d = decode_frame(build_identification(icao, cs))
        chk(f"ident {icao} kind", d["kind"] == "ident", d["kind"])
        chk(f"ident {icao} address", d["icao"] == icao, d["icao"])
        chk(f"ident {icao} callsign", d["callsign"] == cs, d["callsign"])


def test_adsb_position():
    print("\naircraft_emulator.build_position -> decode_frame (raw CPR)")
    icao = "4CA1FE"
    d = decode_frame(build_position(icao, 51.4775, -0.4614, 35000, False))
    chk("position kind", d["kind"] == "position", d["kind"])
    chk("position returns raw CPR, not lat/lon",
        "cpr_lat" in d and "cpr_lon" in d and "lat" not in d and "lon" not in d,
        str(sorted(d)))
    chk("even frame flagged even", d["odd_flag"] is False)
    chk("CPR fields are 17-bit",
        0 <= d["cpr_lat"] < 131072 and 0 <= d["cpr_lon"] < 131072,
        f"{d['cpr_lat']},{d['cpr_lon']}")
    d_odd = decode_frame(build_position(icao, 51.4775, -0.4614, 35000, True))
    chk("odd frame flagged odd", d_odd["odd_flag"] is True)

    # Altitude survives at 25 ft resolution.
    for alt in (-1000, 0, 5000, 18000, 35000, 50000):
        got = decode_frame(build_position(icao, 51.0, -0.4, alt, False))["alt_ft"]
        chk(f"position altitude {alt:>6} ft within 25 ft", near(got, alt, 25),
            f"got {got}")

    # The CPR pair must resolve back to the transmitted position.  This is the
    # real test of the encoder: encode a known point as even+odd, then run the
    # decoder's global-decode over the pair.
    random.seed(7)
    worst = 0.0
    for _ in range(200):
        lat = random.uniform(-70.0, 70.0)
        lon = random.uniform(-179.0, 179.0)
        e = decode_frame(build_position(icao, lat, lon, 35000, False))
        o = decode_frame(build_position(icao, lat, lon, 35000, True))
        res = dec.cpr_resolve(e["cpr_lat"], e["cpr_lon"],
                              o["cpr_lat"], o["cpr_lon"], use_odd=False)
        if res is None:
            continue          # even/odd straddled a zone boundary
        dlat = abs(res[0] - lat)
        dlon = abs(res[1] - lon) * math.cos(math.radians(lat))
        worst = max(worst, math.hypot(dlat, dlon) * 60.0)
    chk("CPR even/odd pair resolves to the transmitted point",
        worst < 0.05, f"worst error {worst * 1852:.0f} m over 200 points")


def test_adsb_velocity():
    print("\naircraft_emulator.build_velocity -> decode_frame")
    icao = "4CA1FE"
    for spd, trk in ((450, 0), (450, 90), (450, 180), (450, 270),
                     (120, 45), (999, 337.4), (250, 200)):
        d = decode_frame(build_velocity(icao, spd, trk, 0))
        chk(f"velocity {spd}kt/{trk}° kind", d["kind"] == "velocity", d["kind"])
        chk(f"velocity {spd}kt/{trk}° speed within 2 kt",
            near(d["speed_kt"], spd, 2), f"got {d['speed_kt']}")
        err = abs(iff.angle_diff(d["track_deg"], trk))
        chk(f"velocity {spd}kt/{trk}° track within 1°", err < 1.0,
            f"got {d['track_deg']:.2f}, err {err:.2f}°")

    # Supersonic switches to the 4 kt/LSB subtype.
    d = decode_frame(build_velocity(icao, 1500, 45, 0))
    chk("supersonic uses subtype 2", d["subtype"] == 2, str(d["subtype"]))
    chk("supersonic speed within 8 kt", near(d["speed_kt"], 1500, 8),
        f"got {d['speed_kt']}")

    # Vertical rate, both signs, at 64 fpm resolution.
    for vr in (-2048, -1800, -64, 0, 64, 1800, 2048):
        d = decode_frame(build_velocity(icao, 450, 90, vr))
        got = d["vrate_fpm"]
        if vr == 0:
            chk("vrate 0 fpm round-trips", got == 0, f"got {got}")
        else:
            chk(f"vrate {vr:>6} fpm within 64", near(got, vr, 64), f"got {got}")


def test_adsb_rejects_malformed():
    print("\naircraft_emulator.decode_frame rejects malformed frames")
    good = build_identification("4CA1FE", "BAW117")

    def raises(raw, label):
        try:
            decode_frame(raw)
        except ValueError:
            chk(f"rejects {label}", True)
        except Exception as e:
            chk(f"rejects {label}", False, f"raised {type(e).__name__}, want ValueError")
        else:
            chk(f"rejects {label}", False, "decoded without error")

    raises("", "an empty frame")
    raises(good[:20], "a truncated frame")
    raises(good + "00", "an over-long frame")
    raises("Z" * 28, "a non-hex frame")
    raises(good[:-2] + ("00" if good[-2:] != "00" else "11"), "a bad CRC")

    # Accepts the dump1090 wire framing the emulator actually emits.
    chk("accepts *HEX; wire framing",
        decode_frame(f"*{good};")["callsign"] == "BAW117")
    chk("accepts bytes input",
        decode_frame(bytes.fromhex(good))["callsign"] == "BAW117")
    chk("accepts lowercase hex",
        decode_frame(good.lower())["callsign"] == "BAW117")


def main():
    test_iff_roundtrip()
    test_iff_rejects_malformed()
    test_adsb_ident()
    test_adsb_position()
    test_adsb_velocity()
    test_adsb_rejects_malformed()
    print()
    if _fails:
        print(f"{len(_fails)} FAILURES:")
        for f in _fails:
            print(f"  - {f}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
