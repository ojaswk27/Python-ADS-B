#!/usr/bin/env python3
"""
ASTERIX CAT021 (ADS-B Target Reports) encoder — minimal subset.

Only the data items needed to convey an emulated airborne target are
implemented.  Items are emitted in UAP (FRN) order and the FSPEC is built
automatically from whichever items are present.

Implemented items (FRN → item):
     1  I021/010  Data Source Identification          (SAC / SIC)
     2  I021/040  Target Report Descriptor
     3  I021/161  Track Number
     6  I021/130  Position in WGS-84 co-ordinates
    11  I021/080  Target Address (ICAO 24-bit)
    16  I021/140  Geometric Height
    21  I021/145  Flight Level
    22  I021/152  Magnetic Heading
    26  I021/160  Airborne Ground Vector (speed + true track)
    29  I021/170  Target Identification (callsign)

All multi-byte fields are big-endian, as ASTERIX is network-byte-order on the
wire.  Reference: EUROCONTROL ASTERIX Part 12, Category 021, Edition 2.x.
"""

import struct

CAT = 21

# 6-bit IA5 charset for I021/170 — same ordering as the ADS-B BDS 0,8 callsign.
_CHARSET = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"


# ── Item encoders ─────────────────────────────────────────────────────────────

def _s24(value: int) -> bytes:
    """Pack a signed integer into 24-bit two's-complement, big-endian."""
    return (value & 0xFFFFFF).to_bytes(3, "big")


def _i010(sac: int, sic: int) -> bytes:
    return bytes((sac & 0xFF, sic & 0xFF))


def _i040() -> bytes:
    # One octet, FX=0.  ATP=000 (24-bit ICAO addr), ARC=00 (25 ft), rest zero.
    return b"\x00"


def _i161(track: int) -> bytes:
    return struct.pack(">H", track & 0x0FFF)        # 4 spare bits + 12-bit track


def _i130(lat_deg: float, lon_deg: float) -> bytes:
    lsb = 180.0 / (1 << 23)                          # ≈ 2.145e-5 deg
    return _s24(round(lat_deg / lsb)) + _s24(round(lon_deg / lsb))


def _i080(icao_hex: str) -> bytes:
    return _s24(int(icao_hex, 16) & 0xFFFFFF)


def _i140(height_ft: float) -> bytes:
    raw = max(-32768, min(32767, round(height_ft / 6.25)))   # LSB = 6.25 ft
    return struct.pack(">h", raw)


def _i145(alt_ft: float) -> bytes:
    raw = max(-32768, min(32767, round(alt_ft / 25.0)))      # LSB = 1/4 FL = 25 ft
    return struct.pack(">h", raw)


def _i152(mag_hdg_deg: float) -> bytes:
    lsb = 360.0 / (1 << 16)
    return struct.pack(">H", round((mag_hdg_deg % 360.0) / lsb) & 0xFFFF)


def _i160(speed_kt: float, track_deg: float) -> bytes:
    gs  = round((speed_kt / 3600.0) / (2 ** -14)) & 0x7FFF    # LSB 2^-14 NM/s
    trk = round((track_deg % 360.0) / (360.0 / (1 << 16))) & 0xFFFF
    return struct.pack(">HH", gs, trk)               # RE bit (bit16) left 0


def _i170(callsign: str) -> bytes:
    cs = (callsign.upper() + "        ")[:8]
    bits = 0
    for ch in cs:
        idx = _CHARSET.find(ch)
        bits = (bits << 6) | (idx if idx >= 0 else 0)
    return bits.to_bytes(6, "big")                   # 8 chars × 6 bits = 48 bits


# ── Record / message assembly ─────────────────────────────────────────────────

def build_record(items: dict) -> bytes:
    """Build one CAT021 record from {FRN: payload_bytes}.

    The FSPEC is generated automatically: one bit per FRN (MSB = FRN1), seven
    FRNs per octet, with the low bit (FX) set on every octet but the last.
    """
    max_frn = max(items)
    n_oct   = (max_frn + 6) // 7
    fspec   = bytearray(n_oct)
    for frn in items:
        fspec[(frn - 1) // 7] |= 1 << (7 - (frn - 1) % 7)
    for i in range(n_oct - 1):
        fspec[i] |= 0x01                             # FX — another octet follows
    body = b"".join(items[frn] for frn in sorted(items))
    return bytes(fspec) + body


def build_message(records: list) -> bytes:
    """Wrap one or more records into a CAT021 ASTERIX data block."""
    body   = b"".join(records)
    length = 3 + len(body)                           # CAT(1) + LEN(2) + records
    return struct.pack(">BH", CAT, length) + body


def target_record(sac: int, sic: int, track: int, icao: str, callsign: str,
                  lat: float, lon: float, alt_ft: float, speed_kt: float,
                  track_deg: float, mag_hdg_deg: float) -> bytes:
    """Build a CAT021 record for one airborne target."""
    return build_record({
        1:  _i010(sac, sic),
        2:  _i040(),
        3:  _i161(track),
        6:  _i130(lat, lon),
        11: _i080(icao),
        16: _i140(alt_ft),
        21: _i145(alt_ft),
        22: _i152(mag_hdg_deg),
        26: _i160(speed_kt, track_deg),
        29: _i170(callsign),
    })
