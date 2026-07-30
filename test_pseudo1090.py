#!/usr/bin/env python3
"""
Custom 1090 format — codec and config validation tests
======================================================
No display needed.

    python test_pseudo1090.py

Two halves:
  * every encoding round-trips across its whole range, at its declared
    resolution, including the awkward ends (0, full scale, negative, clamping)
  * every config mistake produces an error that names the section and field,
    because the config is meant to be edited by someone who is not reading
    pseudo1090.py
"""

import os
import sys
import tempfile

import pseudo1090 as ps

_fails = []


def chk(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"   {detail}" if detail else ""))
    if not cond:
        _fails.append(name)


def _cfg(body, name="t.cfg"):
    d = tempfile.mkdtemp()
    p = os.path.join(d, name)
    with open(p, "w") as fh:
        fh.write(body)
    return p


class AC:
    """Minimal stand-in for SimAircraft."""
    def __init__(self, **kw):
        self.lat, self.lon = 51.5, -0.4
        self.alt_ft, self.speed_kt, self.vrate_fpm = 35000.0, 450.0, 0.0
        self.callsign, self.modes_addr = "TEST123", 0x4CA1FE
        self.mode1, self.mode2, self.mode3a = 0o73, 0o1234, 0o7700
        self.spi_until = 0
        self._hdg = 90.0
        self.__dict__.update(kw)

    def heading(self):
        return self._hdg


# ══ encodings ═════════════════════════════════════════════════════════════════

def test_encodings():
    print("\nencodings round-trip within their declared resolution")
    F = ps.Field

    # uint with a step
    f = F("a", 0, 11, "uint", 25, "ac.alt_ft", None, "t")
    for v in (0, 25, 1000, 35000, 51175):
        got = f.decode(f.encode(v))
        chk(f"uint:25 {v} -> {got}", abs(got - v) <= 25)
    chk("uint clamps above full scale", f.decode(f.encode(10**9)) == 2047 * 25)
    chk("uint clamps below zero", f.decode(f.encode(-500)) == 0)

    # signed int
    f = F("b", 0, 11, "int", 32, "ac.vrate_fpm", None, "t")
    for v in (-32768, -1800, -32, 0, 32, 1800, 32736):
        got = f.decode(f.encode(v))
        chk(f"int:32 {v:>7} -> {got:>7}", abs(got - v) <= 32)
    chk("int clamps at the negative rail", f.decode(f.encode(-10**6)) == -1024 * 32)
    chk("int clamps at the positive rail", f.decode(f.encode(10**6)) == 1023 * 32)

    # scaled
    f = F("c", 0, 24, "scaled", 90, "ac.lat", None, "t")
    res = f.resolution()
    chk("scaled:90/24bit resolution is ~1.1e-5 deg", 1.0e-5 < res < 1.1e-5,
        f"{res:g}")
    worst = 0.0
    v = -90.0
    while v <= 90.0:
        worst = max(worst, abs(f.decode(f.encode(v)) - v))
        v += 0.37
    chk("scaled round-trips across the full range", worst <= res,
        f"worst {worst:g} <= res {res:g}")
    chk("scaled handles exactly zero", f.decode(f.encode(0.0)) == 0.0)
    chk("scaled clamps beyond range", abs(f.decode(f.encode(200.0)) - 90.0) <= res)
    chk("scaled clamps beyond -range", abs(f.decode(f.encode(-200.0)) + 90.0) <= res)

    # ascii6
    f = F("d", 0, 48, "ascii6", None, "ac.callsign", None, "t")
    for s in ("", "A", "BAW117", "TEST123", "ABCDEFGH", "N123AB"):
        chk(f"ascii6 {s!r} round-trips", f.decode(f.encode(s)) == s)
    chk("ascii6 truncates past its width",
        f.decode(f.encode("TOOLONGCALLSIGN")) == "TOOLONGC")
    # An illegal character maps to charset index 0, which is "#" — the ADS-B
    # "not used" symbol — and decode strips those.  Same convention as
    # aircraft_emulator._char_idx and iff_protocol.pack_callsign_bds20.
    chk("ascii6 drops an illegal char rather than corrupting the rest",
        f.decode(f.encode("A!B")) == "AB", repr(f.decode(f.encode("A!B"))))

    # octal, flag, const
    f = F("e", 0, 12, "octal", None, "ac.mode3a", None, "t")
    bad = [v for v in range(0o10000) if f.decode(f.encode(v)) != v]
    chk("all 4096 octal codes round-trip", not bad, f"{len(bad)} bad")
    chk("octal formats as octal", f.format(0o7700) == "7700")

    f = F("g", 0, 1, "flag", None, "ac.spi", None, "t")
    chk("flag True", f.decode(f.encode(True)) is True)
    chk("flag False", f.decode(f.encode(False)) is False)

    f = F("h", 0, 4, "const", 7, None, None, "t")
    chk("const ignores the value", f.decode(f.encode(None)) == 7)

    # None is a safe zero rather than a crash
    f = F("i", 0, 11, "uint", 1, "ac.speed_kt", None, "t")
    chk("None encodes as zero", f.decode(f.encode(None)) == 0)


# ══ frame level ═══════════════════════════════════════════════════════════════

_GOOD = """
[format]
mode = custom
type_field    = 0 4
address_field = 4 24
[msg:position]
id = 1
period = 0.5
lat    = 28 24 scaled:90  ac.lat
lon    = 52 25 scaled:180 ac.lon
alt_ft = 77 11 uint:25    ac.alt_ft
[msg:ident]
id = 2
period = 5.0
callsign = 28 48 ascii6 ac.callsign
squawk   = 76 12 octal  ac.mode3a
"""


def test_frames():
    print("\nframe level: container, CRC, discriminator, address")
    import adsb_decoder as dec
    fmt = ps.load(_cfg(_GOOD))
    ac = AC()

    for name in ("position", "ident"):
        frame = fmt.encode(name, ac)
        chk(f"{name} frame is 28 hex chars", len(frame) == 28, f"{len(frame)}")
        chk(f"{name} frame passes CRC-24", dec.crc_valid(frame))
        got, vals = fmt.decode(frame)
        chk(f"{name} decodes as itself", got == name, got)
        chk(f"{name} carries the address",
            vals.get("address") == ac.modes_addr,
            f"{vals.get('address')}")
        chk(f"{name} address_of resolves",
            fmt.address_of(got, vals) == ac.modes_addr)

    # the discriminator really is what selects the message
    pos, idt = fmt.encode("position", ac), fmt.encode("ident", ac)
    chk("different message types produce different type nibbles",
        pos[0] != idt[0], f"{pos[0]} vs {idt[0]}")

    # a standard ADS-B frame must not silently decode as custom
    from aircraft_emulator import build_position
    std = build_position("4CA1FE", 51.5, -0.4, 35000, False)
    try:
        n, v = fmt.decode(std)
        chk("a standard ADS-B frame does not decode as custom", False,
            f"decoded as {n}")
    except ps.DecodeError:
        chk("a standard ADS-B frame does not decode as custom", True)

    # corruption is caught by the CRC
    bad = ("0" if pos[6] != "0" else "1")
    corrupt = pos[:6] + bad + pos[7:]
    try:
        fmt.decode(corrupt)
        chk("a corrupted frame is rejected by CRC", False, "decoded anyway")
    except ps.DecodeError as e:
        chk("a corrupted frame is rejected by CRC", "CRC" in str(e), str(e))

    for junk, why in ((b"", "empty"), ("ZZ" * 14, "non-hex"),
                      (pos[:20], "short"), (pos + "00", "long")):
        try:
            fmt.decode(junk)
            chk(f"rejects a {why} frame", False, "decoded anyway")
        except ps.DecodeError:
            chk(f"rejects a {why} frame", True)

    chk("accepts bytes input",
        fmt.decode(bytes.fromhex(pos))[0] == "position")
    chk("accepts *HEX; wire framing", fmt.decode(f"*{pos};")[0] == "position")

    # a single-message format needs no discriminator
    solo = ps.load(_cfg("""
[format]
[msg:only]
period = 1.0
alt_ft = 0 11 uint:25 ac.alt_ft
"""))
    chk("a single message type needs no type_field",
        solo.decode(solo.encode("only", ac))[0] == "only")
    chk("...and reports having no address", not solo.has_address)
    chk("...so address_of gives None",
        solo.address_of("only", solo.decode(solo.encode("only", ac))[1]) is None)


# ══ config validation ═════════════════════════════════════════════════════════

def test_validation():
    print("\nconfig errors name the section and field, and all appear at once")

    def problems(body):
        f, errs = ps.load(_cfg(body), strict=False)
        return errs

    cases = [
        ("field past the payload",
         "[format]\n[msg:a]\nx = 80 12 uint:1 ac.alt_ft\n",
         "run past"),
        ("overlapping fields",
         "[format]\n[msg:a]\nx = 0 10 uint:1 ac.alt_ft\ny = 5 10 uint:1 ac.speed_kt\n",
         "overlaps"),
        ("unknown encoding",
         "[format]\n[msg:a]\nx = 0 8 wobble ac.alt_ft\n",
         "unknown encoding"),
        ("unknown source",
         "[format]\n[msg:a]\nx = 0 8 uint:1 ac.nope\n",
         "unknown source"),
        ("missing source",
         "[format]\n[msg:a]\nx = 0 8 uint:1\n",
         "no source"),
        ("ascii6 width not a multiple of 6",
         "[format]\n[msg:a]\nx = 0 7 ascii6 ac.callsign\n",
         "multiple of 6"),
        ("flag wider than one bit",
         "[format]\n[msg:a]\nx = 0 3 flag ac.spi\n",
         "must be 1 bit"),
        ("scaled with no range",
         "[format]\n[msg:a]\nx = 0 12 scaled ac.lat\n",
         "needs a range"),
        ("const with no value",
         "[format]\n[msg:a]\nx = 0 4 const ac.lat\n",
         "needs a value"),
        ("two messages but no type_field",
         "[format]\n[msg:a]\nx = 0 8 uint:1 ac.alt_ft\n"
         "[msg:b]\ny = 0 8 uint:1 ac.speed_kt\n",
         "type_field is required"),
        ("duplicate message ids",
         "[format]\ntype_field = 0 4\n[msg:a]\nid = 1\nx = 4 8 uint:1 ac.alt_ft\n"
         "[msg:b]\nid = 1\ny = 4 8 uint:1 ac.speed_kt\n",
         "already used by"),
        ("id too large for the type_field",
         "[format]\ntype_field = 0 2\n[msg:a]\nid = 99\nx = 4 8 uint:1 ac.alt_ft\n"
         "[msg:b]\nid = 1\ny = 4 8 uint:1 ac.speed_kt\n",
         "does not fit"),
        ("field over the type_field",
         "[format]\ntype_field = 0 4\n[msg:a]\nid = 1\nx = 2 8 uint:1 ac.alt_ft\n"
         "[msg:b]\nid = 2\ny = 20 8 uint:1 ac.speed_kt\n",
         "overlaps the type_field"),
        ("field over the address_field",
         "[format]\naddress_field = 4 24\n[msg:a]\nx = 10 8 uint:1 ac.alt_ft\n",
         "overlaps the shared address_field"),
        ("address_field too narrow",
         "[format]\naddress_field = 4 12\n[msg:a]\nx = 40 8 uint:1 ac.alt_ft\n",
         "needs 24"),
        ("no messages at all",
         "[format]\nmode = custom\n",
         "nothing to transmit"),
        ("jitter larger than period",
         "[format]\n[msg:a]\nperiod = 0.5\njitter = 2.0\nx = 0 8 uint:1 ac.alt_ft\n",
         "must be smaller than period"),
        ("negative period",
         "[format]\n[msg:a]\nperiod = -1\nx = 0 8 uint:1 ac.alt_ft\n",
         "must be > 0"),
        ("unknown section",
         "[format]\n[wat]\nx = 1\n[msg:a]\ny = 0 8 uint:1 ac.alt_ft\n",
         "unknown section"),
        ("bad role",
         "[format]\n[msg:a]\nx = 0 8 uint:1 ac.alt_ft role=captain\n",
         "role must be"),
    ]
    for label, body, needle in cases:
        errs = problems(body)
        hit = any(needle in e for e in errs)
        chk(f"catches {label}", hit,
            "" if hit else f"got {errs}")

    # every problem at once, not just the first
    many = ("[format]\ntype_field = 0 4\n"
            "[msg:a]\nid = 1\nx = 4 8 wobble ac.alt_ft\ny = 90 8 uint:1 ac.lat\n"
            "[msg:b]\nid = 1\nz = 4 7 ascii6 ac.callsign\nw = 4 3 flag ac.spi\n")
    errs = problems(many)
    chk("reports many problems in one pass", len(errs) >= 5, f"{len(errs)} errors")
    chk("every error names its section",
        all(e.startswith("[") for e in errs), str(errs[:2]))

    # a good config raises nothing and loads
    fmt = ps.load(_cfg(_GOOD))
    chk("the good config loads", fmt is not None)
    chk("mode is read", fmt.mode == "custom", fmt.mode)
    chk("describe() works", "position" in fmt.describe())

    # a missing file is a clear error, not a silent default
    try:
        ps.load("/nonexistent/nope.cfg")
        chk("a missing config is an error", False, "loaded anyway")
    except ps.ConfigError as e:
        chk("a missing config is an error", "not found" in str(e))

    # the shipped config must be valid, and must not change default behaviour
    shipped = ps.load(ps.DEFAULT_CFG)
    chk("the shipped pseudo1090.cfg is valid", shipped is not None)
    chk("the shipped config defaults to standard ADS-B",
        shipped.mode == ps.MODE_STANDARD, shipped.mode)
    chk("the shipped config declares an address", shipped.has_address)


def main():
    test_encodings()
    test_frames()
    test_validation()
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
