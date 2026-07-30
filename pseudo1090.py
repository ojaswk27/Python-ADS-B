#!/usr/bin/env python3
"""
Custom 1090 MHz message format — config-driven codec
====================================================
Reuses the ADS-B physical layer so no new hardware is needed: the same 1090 MHz
transceivers, the same 112-bit Mode S frame, the same CRC-24.  Only the
*content* changes — all 88 payload bits are defined by `pseudo1090.cfg` rather
than by DO-260B.

    frame = [ 88 payload bits, entirely yours ][ 24-bit CRC ]
              bit 0 ................. bit 87

Because the payload owns all 88 bits, there is no standard header to lean on.
Two consequences fall out of that, and both are enforced here rather than left
to surprise you at runtime:

  * A decoder cannot tell one message type from another without help.  If the
    config defines more than one [msg:...] section it must also declare
    `type_field` in [format], and every message must declare a unique `id`.
  * Nothing correlates a custom frame with an IFF track unless the config says
    which field carries the aircraft address.  Mark it `role = address`.

Editing the config
------------------
Field lines are positional:

    <name> = <offset> <width> <encoding> <source> [role=...]

  offset    first bit, 0-87, counted from the MSB of the payload
  width     number of bits
  encoding  see ENCODINGS below
  source    where the value comes from: an `ac.*` telemetry name, or
            `const:<n>` for a literal

Check a config without launching the GUI:

    python pseudo1090.py --check
    python pseudo1090.py --check myformat.cfg
    python pseudo1090.py --demo            # encode + decode a sample of each
"""

import argparse
import configparser
import math
import os
import sys

from aircraft_emulator import _sign_crc
import adsb_decoder as _dec

PAYLOAD_BITS = 88
FRAME_HEX_LEN = 28

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(_HERE, "pseudo1090.cfg")

# Transmit modes.  "both" is for comparing the two formats side by side.
MODE_STANDARD = "standard"
MODE_CUSTOM = "custom"
MODE_BOTH = "both"
MODES = (MODE_STANDARD, MODE_CUSTOM, MODE_BOTH)


class ConfigError(Exception):
    """A problem with the config file.  Message names the section and field."""


class DecodeError(Exception):
    """A frame that does not decode under this format."""


# ── 6-bit character set (same alphabet ADS-B and BDS 2,0 use) ─────────────────

_CHARSET = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"


# ── Telemetry sources ─────────────────────────────────────────────────────────
#
# What an aircraft can supply to a field.  Keeping this an explicit table means
# a typo in the config is an error naming the valid options, not a silent zero.

SOURCES = {
    "ac.lat":        lambda ac: ac.lat,
    "ac.lon":        lambda ac: ac.lon,
    "ac.alt_ft":     lambda ac: ac.alt_ft,
    "ac.speed_kt":   lambda ac: ac.speed_kt,
    "ac.track":      lambda ac: ac.heading(),
    "ac.heading":    lambda ac: ac.heading(),
    "ac.vrate_fpm":  lambda ac: ac.vrate_fpm,
    "ac.callsign":   lambda ac: ac.callsign,
    "ac.modes_addr": lambda ac: ac.modes_addr,
    "ac.mode1":      lambda ac: ac.mode1,
    "ac.mode2":      lambda ac: ac.mode2,
    "ac.mode3a":     lambda ac: ac.mode3a,
    "ac.spi":        lambda ac: 1 if getattr(ac, "spi_until", 0) else 0,
}

ENCODINGS = ("uint", "int", "scaled", "ascii6", "octal", "flag", "const")


# ── Field ─────────────────────────────────────────────────────────────────────

class Field:
    """One bit-field: where it lives, how it is encoded, where its value comes
    from.  encode()/decode() are exact inverses within the field's resolution."""

    __slots__ = ("name", "offset", "width", "enc", "param", "source", "role",
                 "section")

    def __init__(self, name, offset, width, enc, param, source, role, section):
        self.name = name
        self.offset = offset
        self.width = width
        self.enc = enc
        self.param = param
        self.source = source
        self.role = role
        self.section = section

    # -- resolution helpers ------------------------------------------------

    @property
    def span(self):
        return (self.offset, self.offset + self.width - 1)

    def resolution(self):
        """Smallest change this field can represent, for the --demo report."""
        if self.enc in ("uint", "int", "octal"):
            return self.param or 1
        if self.enc == "scaled":
            return self.param / float((1 << (self.width - 1)) - 1)
        return None

    # -- value <-> raw bits ------------------------------------------------

    def encode(self, value):
        """Python value -> the unsigned integer that goes in the bits."""
        mask = (1 << self.width) - 1

        if self.enc == "const":
            return int(self.param) & mask

        if self.enc == "flag":
            return 1 if value else 0

        if self.enc == "ascii6":
            text = (str(value or "").upper())[: self.width // 6]
            text = text.ljust(self.width // 6)
            bits = 0
            for ch in text:
                idx = _CHARSET.find(ch)
                bits = (bits << 6) | (idx if idx >= 0 else 0)
            return bits & mask

        if value is None:
            return 0

        if self.enc in ("uint", "octal"):
            scale = self.param or 1
            return max(0, min(int(round(float(value) / scale)), mask))

        if self.enc == "int":
            scale = self.param or 1
            lim = 1 << (self.width - 1)
            raw = int(round(float(value) / scale))
            raw = max(-lim, min(raw, lim - 1))
            return raw & mask

        if self.enc == "scaled":
            lim = (1 << (self.width - 1)) - 1
            frac = float(value) / self.param
            frac = max(-1.0, min(frac, 1.0))
            return int(round(frac * lim)) & mask

        raise ConfigError(f"[{self.section}] {self.name}: unknown encoding "
                          f"{self.enc!r}")

    def decode(self, raw):
        """The unsigned integer from the bits -> Python value."""
        if self.enc == "const":
            return raw

        if self.enc == "flag":
            return bool(raw)

        if self.enc == "ascii6":
            n = self.width // 6
            chars = [_CHARSET[(raw >> (6 * (n - 1 - i))) & 0x3F] for i in range(n)]
            return "".join(chars).replace("#", "").strip()

        if self.enc in ("uint", "octal"):
            return raw * (self.param or 1)

        if self.enc == "int":
            lim = 1 << (self.width - 1)
            signed = raw - (1 << self.width) if raw >= lim else raw
            return signed * (self.param or 1)

        if self.enc == "scaled":
            lim = (1 << (self.width - 1)) - 1
            signed = raw - (1 << self.width) if raw >= (1 << (self.width - 1)) else raw
            return signed / float(lim) * self.param

        raise ConfigError(f"[{self.section}] {self.name}: unknown encoding "
                          f"{self.enc!r}")

    def format(self, value):
        """Human-readable rendering for the log and the decoded pane."""
        if value is None:
            return "—"
        if self.enc == "octal":
            return f"{int(value):o}"
        if self.enc == "flag":
            return "yes" if value else "no"
        if self.enc == "ascii6":
            return str(value)
        if self.enc == "scaled":
            return f"{value:+.5f}"
        if isinstance(value, float):
            return f"{value:g}"
        return str(value)


# ── Message ───────────────────────────────────────────────────────────────────

class Message:
    """One message type: its fields and its transmit schedule."""

    __slots__ = ("name", "msg_id", "period", "jitter", "fields")

    def __init__(self, name, msg_id, period, jitter, fields):
        self.name = name
        self.msg_id = msg_id
        self.period = period
        self.jitter = jitter
        self.fields = fields

    def field(self, name):
        for f in self.fields:
            if f.name == name:
                return f
        return None


# ── Format ────────────────────────────────────────────────────────────────────

class Format:
    """A whole custom format: the container, the discriminator, and the
    messages.  Built by load(); validate() is called for you."""

    def __init__(self, path, mode, type_field, addr_field, messages,
                 address_msg_field):
        self.path = path
        self.mode = mode
        self.type_field = type_field          # (offset, width) or None
        self.addr_field = addr_field          # (offset, width) or None — shared
        self.messages = messages              # dict name -> Message
        self._addr = address_msg_field        # (msg_name, field_name) or None

    # -- introspection -----------------------------------------------------

    @property
    def has_address(self):
        """Whether decoded messages can be tied to an aircraft at all."""
        return self.addr_field is not None or self._addr is not None

    def describe(self):
        """A short human summary, for --check and the UI."""
        out = [f"{os.path.basename(self.path)}  mode={self.mode}"]
        if self.type_field:
            out.append(f"type_field: bits {self.type_field[0]}"
                       f"..{self.type_field[0] + self.type_field[1] - 1}")
        else:
            out.append("type_field: none (single message type)")
        if self.addr_field:
            o, w = self.addr_field
            out.append(f"address:    bits {o}..{o + w - 1}, in every message")
        elif self._addr:
            out.append(f"address:    {'.'.join(self._addr)} — only [msg:"
                       f"{self._addr[0]}] can be correlated")
        else:
            out.append("address:    NONE — decoded messages cannot be tied "
                       "to a track")
        for m in self.messages.values():
            used = sum(f.width for f in m.fields)
            out.append(f"  [msg:{m.name}] id={m.msg_id} "
                       f"period={m.period}±{m.jitter}s  "
                       f"{len(m.fields)} fields, {used}/{PAYLOAD_BITS} bits")
            for f in m.fields:
                lo, hi = f.span
                res = f.resolution()
                res_s = f"  res {res:g}" if res else ""
                out.append(f"      {f.name:<12} bits {lo:>2}-{hi:<2} "
                           f"{f.enc}{'' if f.param is None else ':' + str(f.param)}"
                           f"  <- {f.source}{res_s}")
        return "\n".join(out)

    # -- codec -------------------------------------------------------------

    def encode(self, msg_name, ac):
        """Build one frame for `msg_name` from aircraft `ac`.  Returns 28 hex
        characters: 88 payload bits packed per the config, plus CRC-24."""
        msg = self.messages[msg_name]
        payload = 0
        for f in msg.fields:
            if f.enc == "const":
                value = None
            elif f.source in SOURCES:
                value = SOURCES[f.source](ac)
            else:
                raise ConfigError(f"[msg:{msg_name}] {f.name}: unknown source "
                                  f"{f.source!r}")
            raw = f.encode(value) & ((1 << f.width) - 1)
            shift = PAYLOAD_BITS - f.offset - f.width
            payload |= raw << shift

        # The discriminator and the shared address are written from the format,
        # not from a per-message source.
        for span, value in ((self.type_field, msg.msg_id),
                            (self.addr_field, getattr(ac, "modes_addr", 0))):
            if span is None:
                continue
            off, wid = span
            shift = PAYLOAD_BITS - off - wid
            mask = ((1 << wid) - 1) << shift
            payload = (payload & ~mask) | ((int(value or 0) & ((1 << wid) - 1)) << shift)

        return _sign_crc(f"{payload:022X}")

    def decode(self, frame):
        """Decode one 28-char hex frame.  Returns (msg_name, {field: value}).

        Raises DecodeError for a frame that is the wrong length, fails CRC, or
        carries a type id this format does not define — which is exactly what
        should happen to a standard ADS-B frame fed in here, and to a corrupted
        one.
        """
        if isinstance(frame, (bytes, bytearray)):
            frame = frame.hex().upper()
        f = str(frame).strip().upper().lstrip("*").rstrip(";")
        if len(f) != FRAME_HEX_LEN:
            raise DecodeError(f"expected {FRAME_HEX_LEN} hex chars, got {len(f)}")
        try:
            n = int(f, 16)
        except ValueError:
            raise DecodeError(f"not hex: {f!r}") from None
        if not _dec.crc_valid(f):
            raise DecodeError("CRC-24 invalid")

        payload = n >> 24

        if self.type_field is not None:
            off, wid = self.type_field
            shift = PAYLOAD_BITS - off - wid
            got = (payload >> shift) & ((1 << wid) - 1)
            msg = next((m for m in self.messages.values() if m.msg_id == got), None)
            if msg is None:
                raise DecodeError(f"no message defined for type id {got}")
        else:
            msg = next(iter(self.messages.values()))

        out = {}
        if self.addr_field is not None:
            off, wid = self.addr_field
            shift = PAYLOAD_BITS - off - wid
            out["address"] = (payload >> shift) & ((1 << wid) - 1)
        for fld in msg.fields:
            shift = PAYLOAD_BITS - fld.offset - fld.width
            raw = (payload >> shift) & ((1 << fld.width) - 1)
            out[fld.name] = fld.decode(raw)
        return msg.name, out

    def address_of(self, msg_name, values):
        """The aircraft address a decoded message correlates to, or None if the
        config never declared one for this message type."""
        if self.addr_field is not None:
            v = values.get("address")
            return None if v is None else int(v)
        if self._addr is None or self._addr[0] != msg_name:
            return None
        v = values.get(self._addr[1])
        return None if v is None else int(v)


# ── Loading ───────────────────────────────────────────────────────────────────

def _parse_field(section, name, spec):
    """`<offset> <width> <encoding> <source> [role=x]` -> Field."""
    parts = spec.split()
    if len(parts) < 3:
        raise ConfigError(
            f"[{section}] {name}: expected "
            f"'<offset> <width> <encoding> <source> [role=...]', got {spec!r}")

    try:
        offset = int(parts[0])
        width = int(parts[1])
    except ValueError:
        raise ConfigError(f"[{section}] {name}: offset and width must be "
                          f"integers, got {parts[0]!r} {parts[1]!r}") from None

    enc_tok = parts[2]
    enc, _, param_s = enc_tok.partition(":")
    if enc not in ENCODINGS:
        raise ConfigError(f"[{section}] {name}: unknown encoding {enc!r}. "
                          f"Valid: {', '.join(ENCODINGS)}")
    param = None
    if param_s:
        try:
            param = float(param_s) if "." in param_s else int(param_s)
        except ValueError:
            raise ConfigError(f"[{section}] {name}: encoding parameter must be "
                              f"a number, got {param_s!r}") from None

    role = None
    source = None
    for tok in parts[3:]:
        if tok.startswith("role="):
            role = tok[5:]
        elif source is None:
            source = tok

    if enc == "const":
        if param is None:
            raise ConfigError(f"[{section}] {name}: const needs a value, "
                              f"e.g. const:7")
    elif source is None:
        raise ConfigError(f"[{section}] {name}: no source given. Use one of "
                          f"{', '.join(sorted(SOURCES))} or const:<n>")
    elif source not in SOURCES:
        raise ConfigError(f"[{section}] {name}: unknown source {source!r}. "
                          f"Valid: {', '.join(sorted(SOURCES))}")

    if role not in (None, "address", "msgtype"):
        raise ConfigError(f"[{section}] {name}: role must be 'address' or "
                          f"'msgtype', got {role!r}")

    return Field(name, offset, width, enc, param, source, role, section)


def _validate(fmt, messages):
    """Collect every problem at once, so a single --check run fixes the file."""
    errs = []

    for m in messages.values():
        for f in m.fields:
            if f.width < 1:
                errs.append(f"[msg:{m.name}] {f.name}: width must be >= 1")
                continue
            if f.offset < 0:
                errs.append(f"[msg:{m.name}] {f.name}: offset must be >= 0")
            if f.offset + f.width > PAYLOAD_BITS:
                errs.append(
                    f"[msg:{m.name}] {f.name}: bits {f.offset}-"
                    f"{f.offset + f.width - 1} run past the {PAYLOAD_BITS}-bit "
                    f"payload (last usable bit is {PAYLOAD_BITS - 1})")
            if f.enc == "ascii6" and f.width % 6:
                errs.append(f"[msg:{m.name}] {f.name}: ascii6 width must be a "
                            f"multiple of 6, got {f.width}")
            if f.enc == "scaled" and not f.param:
                errs.append(f"[msg:{m.name}] {f.name}: scaled needs a range, "
                            f"e.g. scaled:90")
            if f.enc == "flag" and f.width != 1:
                errs.append(f"[msg:{m.name}] {f.name}: flag must be 1 bit, "
                            f"got {f.width}")

        # overlapping fields
        for i, a in enumerate(m.fields):
            for b in m.fields[i + 1:]:
                if a.offset < b.offset + b.width and b.offset < a.offset + a.width:
                    errs.append(
                        f"[msg:{m.name}] {a.name} (bits {a.span[0]}-{a.span[1]}) "
                        f"overlaps {b.name} (bits {b.span[0]}-{b.span[1]})")

    # discriminator
    if len(messages) > 1:
        if fmt["type_field"] is None:
            errs.append(
                f"[format] type_field is required: {len(messages)} message "
                f"types are defined and nothing distinguishes them on the wire. "
                f"Add e.g. 'type_field = 0 4' and give each [msg:...] an 'id'.")
        else:
            off, wid = fmt["type_field"]
            if off + wid > PAYLOAD_BITS:
                errs.append(f"[format] type_field bits {off}-{off + wid - 1} "
                            f"run past the {PAYLOAD_BITS}-bit payload")
            seen = {}
            for m in messages.values():
                if m.msg_id is None:
                    errs.append(f"[msg:{m.name}] needs an 'id' (type_field is "
                                f"declared, so every message must be "
                                f"identifiable)")
                elif m.msg_id in seen:
                    errs.append(f"[msg:{m.name}] id={m.msg_id} is already used "
                                f"by [msg:{seen[m.msg_id]}]")
                else:
                    seen[m.msg_id] = m.name
                if m.msg_id is not None and m.msg_id >= (1 << wid):
                    errs.append(f"[msg:{m.name}] id={m.msg_id} does not fit in "
                                f"the {wid}-bit type_field (max "
                                f"{(1 << wid) - 1})")
            # a field must not sit on top of the discriminator
            for m in messages.values():
                for f in m.fields:
                    if f.offset < off + wid and off < f.offset + f.width:
                        errs.append(
                            f"[msg:{m.name}] {f.name} (bits {f.span[0]}-"
                            f"{f.span[1]}) overlaps the type_field "
                            f"(bits {off}-{off + wid - 1})")

    af = fmt.get("address_field")
    if af is not None:
        off, wid = af
        if off + wid > PAYLOAD_BITS:
            errs.append(f"[format] address_field bits {off}-{off + wid - 1} "
                        f"run past the {PAYLOAD_BITS}-bit payload")
        if wid < 24:
            errs.append(f"[format] address_field is {wid} bits; a 24-bit ICAO "
                        f"address needs 24")
        if fmt.get("type_field") is not None:
            toff, twid = fmt["type_field"]
            if off < toff + twid and toff < off + wid:
                errs.append(f"[format] address_field (bits {off}-{off + wid - 1}) "
                            f"overlaps type_field (bits {toff}-{toff + twid - 1})")
        for m in messages.values():
            for f in m.fields:
                if f.offset < off + wid and off < f.offset + f.width:
                    errs.append(
                        f"[msg:{m.name}] {f.name} (bits {f.span[0]}-{f.span[1]}) "
                        f"overlaps the shared address_field "
                        f"(bits {off}-{off + wid - 1})")

    if not messages:
        errs.append("no [msg:...] sections defined — nothing to transmit")

    return errs


def load(path=None, strict=True):
    """Read a config file and return a Format.

    strict=False collects problems instead of raising, for --check.
    """
    path = path or DEFAULT_CFG
    if not os.path.exists(path):
        raise ConfigError(f"config not found: {path}")

    cp = configparser.ConfigParser()
    # Keep field names in the case they were written in.
    cp.optionxform = str
    try:
        cp.read(path)
    except configparser.Error as e:
        raise ConfigError(f"{path}: {e}") from None

    # Errors accumulate rather than raising at the first one, so a single
    # --check run tells you everything that needs fixing.
    errs = []
    fmt = {"mode": MODE_CUSTOM, "type_field": None, "address_field": None}
    if cp.has_section("format"):
        sec = cp["format"]
        mode = sec.get("mode", MODE_CUSTOM).strip().lower()
        if mode not in MODES:
            errs.append(f"[format] mode must be one of {', '.join(MODES)}, "
                        f"got {mode!r}")
        else:
            fmt["mode"] = mode
        for key in ("type_field", "address_field"):
            raw = sec.get(key, "").strip()
            if not raw:
                continue
            parts = raw.split()
            if len(parts) != 2:
                errs.append(f"[format] {key} must be '<offset> <width>', "
                            f"e.g. '{key} = 0 4'")
                continue
            try:
                fmt[key] = (int(parts[0]), int(parts[1]))
            except ValueError:
                errs.append(f"[format] {key} offset and width must be integers")

    reserved = {"id", "period", "jitter"}
    messages = {}
    addr = None
    for section in cp.sections():
        if section == "format":
            continue
        if not section.startswith("msg:"):
            errs.append(f"[{section}] unknown section; expected [format] or "
                        f"[msg:<name>]")
            continue
        name = section[4:].strip()
        sec = cp[section]

        try:
            msg_id = sec.getint("id", fallback=None)
        except ValueError:
            errs.append(f"[{section}] id must be an integer, got "
                        f"{sec.get('id')!r}")
            msg_id = None
        try:
            period = sec.getfloat("period", fallback=1.0)
            jitter = sec.getfloat("jitter", fallback=0.0)
        except ValueError:
            errs.append(f"[{section}] period and jitter must be numbers")
            period, jitter = 1.0, 0.0
        if period <= 0:
            errs.append(f"[{section}] period must be > 0, got {period}")
            period = 1.0
        if jitter < 0:
            errs.append(f"[{section}] jitter must be >= 0, got {jitter}")
            jitter = 0.0
        elif jitter >= period:
            errs.append(f"[{section}] jitter ({jitter}) must be smaller than "
                        f"period ({period})")

        fields = []
        for key in sec:
            if key in reserved:
                continue
            try:
                f = _parse_field(section, key, sec[key])
            except ConfigError as e:
                errs.append(str(e))
                continue
            if f.role == "address":
                if addr is not None:
                    errs.append(
                        f"[{section}] {key}: role=address is already set by "
                        f"{addr[0]}.{addr[1]}; only one field may be the "
                        f"address")
                else:
                    addr = (name, key)
            fields.append(f)

        messages[name] = Message(name, msg_id, period, jitter, fields)

    errs += _validate(fmt, messages)
    if errs:
        if strict:
            raise ConfigError(f"{path}:\n  " + "\n  ".join(errs))
        return None, errs

    f = Format(path, fmt["mode"], fmt["type_field"], fmt["address_field"],
               messages, addr)
    return (f, []) if not strict else f


# ── CLI ───────────────────────────────────────────────────────────────────────

def _check(path):
    fmt, errs = load(path, strict=False)
    if errs:
        print(f"{path or DEFAULT_CFG}: {len(errs)} problem(s)\n")
        for e in errs:
            print(f"  {e}")
        return 1
    print(fmt.describe())
    if not fmt.has_address:
        print("\nWARNING: no address declared, so decoded custom messages "
              "cannot be\n         correlated with IFF tracks.  Add "
              "'address_field = <off> 24' to\n         [format], or "
              "'role=address' to one field.")
    print("\nOK")
    return 0


def _demo(path):
    """Round-trip a synthetic aircraft through every message type."""
    fmt = load(path)

    class _AC:
        lat, lon = 51.60123, -0.40456
        alt_ft, speed_kt, vrate_fpm = 35000.0, 450.0, 1800.0
        callsign, modes_addr = "BAW117", 0x4CA1FE
        mode1, mode2, mode3a = 0o73, 0o1234, 0o7700
        spi_until = 0

        def heading(self):
            return 137.4

    ac = _AC()
    rc = 0
    for name in fmt.messages:
        frame = fmt.encode(name, ac)
        got_name, values = fmt.decode(frame)
        ok = got_name == name
        if fmt.addr_field is not None:
            got_addr = values.get("address")
            if got_addr != ac.modes_addr:
                print(f"    *** address {got_addr:06X} != "
                      f"{ac.modes_addr:06X} ***")
                rc = 1
            else:
                print(f"    {'address':<12} sent {ac.modes_addr:06X}"
                      f"       got {got_addr:06X}")
        print(f"[msg:{name}] -> {frame}   CRC ok, decodes as {got_name!r}"
              f"{'' if ok else '   *** MISMATCH ***'}")
        for fld in fmt.messages[name].fields:
            v = values[fld.name]
            src = (SOURCES[fld.source](ac) if fld.source in SOURCES else fld.param)
            note = ""
            if fld.enc == "ascii6":
                if str(v).strip() != str(src).strip().upper()[: fld.width // 6]:
                    note = f"   *** sent {src!r} got {v!r} ***"
            elif fld.enc == "flag":
                if bool(v) != bool(src):
                    note = f"   *** sent {bool(src)} got {bool(v)} ***"
            else:
                # Numeric: must land within one quantisation step of the input.
                res = fld.resolution() or 1
                drift = abs(float(v) - float(src or 0))
                if drift > res * 1.5:
                    note = f"   *** drift {drift:g} exceeds resolution {res:g} ***"
            if note:
                rc = 1
            # Render both sides through the field's own formatter, so an octal
            # field is not shown as decimal on one side and octal on the other.
            print(f"    {fld.name:<12} sent {fld.format(src):<12} got "
                  f"{fld.format(v):<12}{note}")
        if not ok:
            rc = 1
        print()
    print("OK" if rc == 0 else "FAILURES")
    return rc


def main():
    p = argparse.ArgumentParser(
        description="Custom 1090 MHz message format: validate and round-trip",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("--check", action="store_true",
                   help="validate the config and print the layout")
    p.add_argument("--demo", action="store_true",
                   help="encode and decode a sample of every message type")
    p.add_argument("cfg", nargs="?", default=None,
                   help=f"config file (default: {os.path.basename(DEFAULT_CFG)})")
    args = p.parse_args()
    try:
        if args.demo:
            return _demo(args.cfg)
        return _check(args.cfg)
    except ConfigError as e:
        print(f"config error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
