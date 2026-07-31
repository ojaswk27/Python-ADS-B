"""
IFF reply binary encoder.
"""

import math
import os
import struct
import sys
from dataclasses import dataclass


def _load_magic():
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".formats")
    if d not in sys.path:
        sys.path.insert(0, d)
    try:
        import magic  # type: ignore
        return magic.IFF_REPLY_HEADER_MAGIC
    except Exception:
        return 0

# Constants

HEADER_MAGIC = _load_magic()

MODE_1     = 1
MODE_2     = 2
MODE_3A    = 3
MODE_C     = 4
MODE_S_AC  = 5    # All-Call
MODE_S_SEL = 6    # Selective

MODE_NAMES = {
    MODE_1:     "M1",
    MODE_2:     "M2",
    MODE_3A:    "M3A",
    MODE_C:     "MC",
    MODE_S_AC:  "MS-AC",
    MODE_S_SEL: "MS-SEL",
}

_CLASSIC_MODES  = (MODE_1, MODE_2, MODE_3A, MODE_C)
_MODE_S_MODES   = (MODE_S_AC, MODE_S_SEL)


def mode_is_s(mode):
    return mode in _MODE_S_MODES


def turnaround_us(mode):
    return TURNAROUND_S_US if mode_is_s(mode) else TURNAROUND_AC_US


def range_to_rtt_us(rng_nm, mode):
    return rng_nm * RADAR_MILE_US + turnaround_us(mode)


def rtt_to_range_nm(rtt_us, mode):
    rng = (rtt_us - turnaround_us(mode)) / RADAR_MILE_US
    return round(rng / RANGE_QUANT_NM) * RANGE_QUANT_NM

RANGE_LSB_NM = 1.0 / 128.0
MAX_TARGETS  = 20

C_M_S            = 299_792_458.0
NM_M             = 1852.0
RADAR_MILE_US    = 12.3559        # round-trip microseconds per nautical mile
TURNAROUND_AC_US = 3.0            # Mode 1/2/3A/C transponder reply delay
TURNAROUND_S_US  = 128.0          # Mode S DF11/DF4/DF5/DF20/DF21 reply delay
RANGE_QUANT_NM   = 1.0 / 64.0     # SSR range cell

_RECORD_LEN_CLASSIC = 48
_RECORD_LEN_MODE_S  = 50
_HEADER_LEN         = 10


# Target record

@dataclass
class TargetRecord:
    """One target's contribution to a reply block.

    code:        for Modes 1/2/3-A/C — packed Code field (octal squawk or altitude code)
    modes_addr:  for Mode S — 24-bit ICAO of the responding aircraft
    confidence:  placeholder, defaults to "all confident"
    """
    range_nm:   float
    code:       int = 0
    modes_addr: int = 0
    confidence: int = 0xFFFF


# Encoders

def encode_mode_c(altitude_ft: int) -> int:
    """Pack altitude into the 16-bit Code field for Mode C replies."""
    sign = 0 if altitude_ft >= 0 else 1
    mag  = min(abs(int(altitude_ft)) // 25, 0x7FF)        # 11 bits
    return (sign << 11) | mag


def decode_mode_c(code: int) -> int:
    """Inverse of encode_mode_c.  Lossy to the nearest 25 ft, same as encode."""
    sign = -1 if (code >> 11) & 1 else 1
    mag  = code & 0x7FF
    return sign * mag * 25


# Mode 1 code representation
#
# Mode 1 is a military mission code sent as two pulse groups: A (3 bits, the
# first octal digit, 0-7) and B (2 bits, the second octal digit, 0-3) — 5 bits
# and 32 valid codes in total.  Two representations are in play and they are
# NOT interchangeable:
#
#   display form   two octal digits, 0o00 .. 0o73 with the low digit <= 3.
#                  What the operator types and reads.  Sparse over 0..59.
#   wire form      the packed 5 bits, 0 .. 31.  What the reply carries.
#
# Masking a display-form value with 0x1F silently corrupts everything above
# 0o37 (0o73 -> 0o33), so the conversion has to be explicit in both directions.

MODE_1_MAX_DISPLAY = 0o73


def valid_mode1(v: int) -> bool:
    """True if v is a legal Mode 1 code in display (octal-digit) form."""
    return 0 <= v <= MODE_1_MAX_DISPLAY and (v & 0o7) <= 3


def mode1_to_wire(v: int) -> int:
    """Display form (0o00-0o73) -> packed 5-bit wire form (0-31)."""
    if not valid_mode1(v):
        raise ValueError(
            f"illegal Mode 1 code 0o{v:o}: want 00-73 octal with B digit 0-3")
    return ((v >> 3) << 2) | (v & 0o3)


def mode1_from_wire(bits: int) -> int:
    """Packed 5-bit wire form (0-31) -> display form (0o00-0o73)."""
    bits &= 0x1F
    return ((bits >> 2) << 3) | (bits & 0o3)


# Pressure altitude

STD_QNH_HPA = 1013.25
_FT_PER_HPA = 27.0        # near sea level

# Mode C reporting range: -1000 ft to the top of the Gillham code.
MODE_C_MIN_FT = -1000
MODE_C_MAX_FT = 126700


def encode_mode_c_alt(geo_alt_ft, qnh_hpa=STD_QNH_HPA):
    """Geometric altitude -> the pressure altitude Mode C actually reports.

    Mode C is Gillham-coded in 100 ft steps and is always referenced to
    1013.25 hPa, never to the local QNH.  When the local setting differs, the
    reported figure differs from the aircraft's true height by about 27 ft per
    hPa — which is exactly why the two readouts should not be expected to
    agree, and why a sim that reports geometric altitude at 1 ft resolution is
    telling a comfortable lie.
    """
    press_alt = geo_alt_ft + (STD_QNH_HPA - qnh_hpa) * _FT_PER_HPA
    q = int(round(press_alt / 100.0)) * 100
    return max(MODE_C_MIN_FT, min(MODE_C_MAX_FT, q))


# Special-purpose Mode 3/A codes

SQUAWK_HIJACK   = 0o7500
SQUAWK_RADIO    = 0o7600
SQUAWK_EMERGENCY = 0o7700

EMERGENCY_SQUAWKS = {
    SQUAWK_HIJACK:    "HIJACK",
    SQUAWK_RADIO:     "RADIO FAIL",
    SQUAWK_EMERGENCY: "EMERGENCY",
}


def emergency_label(squawk):
    """Name for a special-purpose squawk, or None for an ordinary code."""
    return EMERGENCY_SQUAWKS.get(squawk)


def _pack_classic(t: TargetRecord) -> bytes:
    rc = int(t.range_nm / RANGE_LSB_NM) & 0xFFFFFFFF
    return struct.pack(">IHH", rc, t.code & 0xFFFF, t.confidence & 0xFFFF) + b"\x00" * 40


def _pack_mode_s(t: TargetRecord) -> bytes:
    rc       = int(t.range_nm / RANGE_LSB_NM) & 0xFFFFFFFF
    addr_b   = (t.modes_addr & 0xFFFFFF).to_bytes(3, "big")
    data14   = addr_b + b"\x00" * 11          # ICAO + ME placeholder
    confid14 = b"\xFF" * 14
    spare18  = b"\x00" * 18
    return struct.pack(">I", rc) + data14 + confid14 + spare18


def build_reply(prt_no: int, azimuth_deg: float, mode: int,
                targets: list) -> bytes:
    """Pack one reply block.  Truncates target list to MAX_TARGETS."""
    if mode not in MODE_NAMES:
        raise ValueError(f"unknown mode {mode}")

    az_counter = int(azimuth_deg * 4096 / 360) & 0xFFF
    n = min(len(targets), MAX_TARGETS)
    header = struct.pack(">IHHBB",
                         HEADER_MAGIC, prt_no & 0xFFFF, az_counter, mode, n)

    pack = _pack_mode_s if mode in _MODE_S_MODES else _pack_classic
    body = b"".join(pack(t) for t in targets[:n])
    return header + body


# Decoder (for the reply log)

def decode_reply(reply: bytes) -> dict:
    """Unpack a reply block into a dict (header + list of per-target dicts)."""
    if len(reply) < _HEADER_LEN:
        raise ValueError(f"reply too short ({len(reply)} B)")
    magic, prt_no, az_counter, mode, n = struct.unpack(">IHHBB", reply[:_HEADER_LEN])
    az_deg = az_counter * 360.0 / 4096.0

    rec_len  = _RECORD_LEN_MODE_S if mode in _MODE_S_MODES else _RECORD_LEN_CLASSIC
    expected = _HEADER_LEN + n * rec_len
    if len(reply) != expected:
        raise ValueError(f"length {len(reply)} != {expected} for mode {mode} n={n}")

    targets = []
    off = _HEADER_LEN
    for _ in range(n):
        rec = reply[off:off + rec_len]
        off += rec_len
        if mode in _MODE_S_MODES:
            rc = struct.unpack(">I", rec[:4])[0]
            addr = int.from_bytes(rec[4:7], "big")
            targets.append({"range_nm": rc * RANGE_LSB_NM, "modes_addr": addr})
        else:
            rc, code, conf = struct.unpack(">IHH", rec[:8])
            targets.append({"range_nm": rc * RANGE_LSB_NM,
                            "code": code, "confidence": conf})

    return {
        "magic":      magic,
        "prt_no":     prt_no,
        "az_deg":     az_deg,
        "az_counter": az_counter,
        "mode":       mode,
        "n":          n,
        "targets":    targets,
    }


def format_decoded(reply: bytes) -> str:
    """One-line human-readable summary for the scanner reply log."""
    d = decode_reply(reply)
    mname = MODE_NAMES.get(d["mode"], f"?{d['mode']}")
    parts = [f"{mname}  az={d['az_deg']:5.1f}°  prt={d['prt_no']}  n={d['n']}"]
    for i, t in enumerate(d["targets"], 1):
        if "modes_addr" in t:
            parts.append(f"[#{i} {t['range_nm']:5.1f}nm {t['modes_addr']:06X}]")
        else:
            parts.append(f"[#{i} {t['range_nm']:5.1f}nm {t['code']:04o}]")
    return "  ".join(parts)


def format_hex(reply: bytes) -> str:
    """Raw hex dump grouped 4 B per word."""
    return reply.hex(" ", 4)


# Geometry helpers (shared by the aircraft-side and radar-side scripts)

def bearing_range_nm(c_lat, c_lon, lat, lon):
    """Bearing (deg from true north) and range (nm) of (lat,lon) from (c_lat,c_lon)."""
    nm_n = (lat - c_lat) * 60.0
    nm_e = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    rng  = math.hypot(nm_e, nm_n)
    brg  = math.degrees(math.atan2(nm_e, nm_n)) % 360.0
    return brg, rng


def angle_diff(a, b):
    """Smallest signed angular difference a-b in [-180, 180]."""
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


# Radar → Aircraft interrogation packet
#
# Private link between the two scripts (not the external 0x3301 control
# message).  Carries the beam geometry so the aircraft-side can decide for
# itself whether it is illuminated, since it is the only side that owns
# aircraft ground truth.

_INTERROGATION_HEAD = struct.Struct(">BHffff")   # mode, prt_no, 4x float32


def pack_interrogation(mode: int, prt_no: int, radar_lat: float, radar_lon: float,
                       beam_az_deg: float, beam_bw_deg: float,
                       selective_addr: int = 0, bds_reg: int = 0) -> bytes:
    head = _INTERROGATION_HEAD.pack(mode & 0xFF, prt_no & 0xFFFF,
                                    radar_lat, radar_lon, beam_az_deg, beam_bw_deg)
    addr = (selective_addr & 0xFFFFFF).to_bytes(3, "big")
    return head + addr + bytes([bds_reg & 0xFF])


def unpack_interrogation(pkt: bytes) -> dict:
    mode, prt_no, radar_lat, radar_lon, beam_az_deg, beam_bw_deg = \
        _INTERROGATION_HEAD.unpack(pkt[:19])
    selective_addr = int.from_bytes(pkt[19:22], "big")
    bds_reg = pkt[22]
    return {
        "mode": mode, "prt_no": prt_no,
        "radar_lat": radar_lat, "radar_lon": radar_lon,
        "beam_az_deg": beam_az_deg, "beam_bw_deg": beam_bw_deg,
        "selective_addr": selective_addr if selective_addr else None,
        "bds_reg": bds_reg,
    }


# Aircraft → Radar per-target reply
#
# Header carries a sim-envelope ICAO (for track correlation — a bookkeeping
# shortcut, not something a real classic-mode interrogation would deliver)
# plus the responder's true position (a shortcut standing in for real IFF's
# time-of-flight ranging).  The payload is mode-specific and only contains
# the field that mode actually carries on air — no cross-mode leakage.
#
# BDS 2,0 (Aircraft Identification / callsign) is the only Mode S register
# implemented; other BDS registers reply address-only.

BDS_CALLSIGN = 0x20   # ICAO Annex 10 BDS register 2,0

# Same 6-bit character alphabet as ADS-B TC=4 / Mode S register 2,0.
_CALLSIGN_CHARSET = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"


def pack_callsign_bds20(callsign: str) -> bytes:
    """Pack an 8-character callsign into the 48-bit (6-byte) content of a
    BDS 2,0 register — 8 characters at 6 bits each, MSB first, exactly as a
    real Mode S Aircraft Identification reply carries it."""
    cs = (callsign or "").upper()[:8]
    cs = cs + " " * (8 - len(cs))
    bits = 0
    for ch in cs:
        idx = _CALLSIGN_CHARSET.find(ch)
        bits = (bits << 6) | (idx if idx >= 0 else 0)
    return bits.to_bytes(6, "big")


def unpack_callsign_bds20(reg6: bytes) -> str:
    """Inverse of pack_callsign_bds20."""
    bits = int.from_bytes(reg6[:6], "big")
    chars = [_CALLSIGN_CHARSET[(bits >> (42 - i * 6)) & 0x3F] for i in range(8)]
    return "".join(chars).replace("#", "").strip()


_REPLY_HEAD = struct.Struct(">BHff")   # mode, prt_no, src_lat, src_lon (after the 3 B icao)


def _pack_reply_header(icao_int: int, mode: int, prt_no: int,
                       src_lat: float, src_lon: float) -> bytes:
    return ((icao_int & 0xFFFFFF).to_bytes(3, "big") +
            _REPLY_HEAD.pack(mode & 0xFF, prt_no & 0xFFFF, src_lat, src_lon))


def _unpack_reply_header(pkt: bytes):
    icao_int = int.from_bytes(pkt[0:3], "big")
    mode, prt_no, src_lat, src_lon = _REPLY_HEAD.unpack(pkt[3:14])
    return icao_int, mode, prt_no, src_lat, src_lon, pkt[14:]


def encode_target_reply(icao_int: int, mode: int, prt_no: int,
                        src_lat: float, src_lon: float, **kw) -> bytes:
    """Build one per-target reply.  Required kwargs depend on mode:
      MODE_1     mission_code in display form (0o00-0o73, B digit 0-3)
      MODE_2     unit_code (12-bit)
      MODE_3A    squawk (12-bit)
      MODE_C     alt_ft
      MODE_S_AC  modes_addr
      MODE_S_SEL modes_addr, bds_reg, callsign (callsign only used if
                 bds_reg == BDS_CALLSIGN)
    """
    header = _pack_reply_header(icao_int, mode, prt_no, src_lat, src_lon)
    if mode == MODE_1:
        payload = bytes([mode1_to_wire(kw["mission_code"])])
    elif mode == MODE_2:
        payload = struct.pack(">H", kw["unit_code"] & 0xFFF)
    elif mode == MODE_3A:
        payload = struct.pack(">H", kw["squawk"] & 0xFFF)
    elif mode == MODE_C:
        payload = struct.pack(">H", encode_mode_c(kw["alt_ft"]))
    elif mode == MODE_S_AC:
        payload = (kw["modes_addr"] & 0xFFFFFF).to_bytes(3, "big")
    elif mode == MODE_S_SEL:
        # addr (3) + BDS code (1) + 48-bit register content (6).  The BDS code
        # byte + register content together form a real 7-byte BDS 2,0 register.
        addr_b  = (kw["modes_addr"] & 0xFFFFFF).to_bytes(3, "big")
        bds_reg = kw.get("bds_reg", 0)
        if bds_reg == BDS_CALLSIGN:
            reg = pack_callsign_bds20(kw.get("callsign"))
        else:
            reg = b"\x00" * 6
        payload = addr_b + bytes([bds_reg & 0xFF]) + reg
    else:
        raise ValueError(f"unknown mode {mode}")
    return header + payload


# Payload length each mode must carry, for validation on decode.
_PAYLOAD_LEN = {
    MODE_1:     1,
    MODE_2:     2,
    MODE_3A:    2,
    MODE_C:     2,
    MODE_S_AC:  3,
    MODE_S_SEL: 10,   # addr(3) + BDS code(1) + register content(6)
}


def decode_target_reply(pkt: bytes) -> dict:
    """Decode one per-target reply.  Only keys the mode actually carries are
    present — callers must not assume e.g. 'callsign' exists for a Mode 3/A
    reply.

    Raises ValueError on a frame that is truncated, carries an unknown mode, or
    whose payload is the wrong length for its mode.  A malformed frame must not
    decode to a partial dict: that is how a receiver ends up displaying fields
    it never actually received.
    """
    if len(pkt) < 14:
        raise ValueError(f"reply too short for a header ({len(pkt)} B, need 14)")
    icao_int, mode, prt_no, src_lat, src_lon, payload = _unpack_reply_header(pkt)
    if mode not in _PAYLOAD_LEN:
        raise ValueError(f"unknown mode {mode}")
    want = _PAYLOAD_LEN[mode]
    if len(payload) != want:
        raise ValueError(f"mode {mode} payload is {len(payload)} B, expected {want}")

    out = {"icao": icao_int, "mode": mode, "prt_no": prt_no,
           "src_lat": src_lat, "src_lon": src_lon}
    if mode == MODE_1:
        out["mission_code"] = mode1_from_wire(payload[0])
    elif mode == MODE_2:
        out["unit_code"] = struct.unpack(">H", payload[:2])[0] & 0xFFF
    elif mode == MODE_3A:
        out["squawk"] = struct.unpack(">H", payload[:2])[0] & 0xFFF
    elif mode == MODE_C:
        out["alt_ft"] = decode_mode_c(struct.unpack(">H", payload[:2])[0])
    elif mode == MODE_S_AC:
        out["modes_addr"] = int.from_bytes(payload[:3], "big")
    elif mode == MODE_S_SEL:
        out["modes_addr"] = int.from_bytes(payload[:3], "big")
        bds_reg = payload[3]
        out["bds_reg"] = bds_reg
        if bds_reg == BDS_CALLSIGN:
            out["callsign"] = unpack_callsign_bds20(payload[4:10])
    return out
