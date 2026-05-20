#!/usr/bin/env python3
"""
ADS-B Decoder
=============
Fully manual implementation of 1090 MHz Mode S Extended Squitter decoding.
Only the Python standard library is used — no external packages.

Usage
-----
    python adsb_decoder.py                     # decode built-in demo messages
    python adsb_decoder.py --msg <28-HEX>      # decode a single message
    python adsb_decoder.py --file msgs.txt     # decode newline-separated hex file
    python adsb_decoder.py --live              # live stream from dump1090 TCP :30002
    python adsb_decoder.py --multicast         # UDP multicast :30003 (group 239.255.0.1)

Frame structure (112 bits / 28 hex chars)
------------------------------------------
  Bits  1–5   : DF  — Downlink Format (17 = ADS-B ES)
  Bits  6–8   : CA  — Capability
  Bits  9–32  : ICAO 24-bit aircraft address
  Bits 33–88  : ME  — Message / Extended Squitter payload (56 bits)
                  Bits 33–37 : Type Code (TC)
                  Bits 38–88 : Type-specific data
  Bits 89–112 : CRC-24

ADS-B message types by Type Code
----------------------------------
  TC  1–4  : Aircraft Identification (callsign + wake vortex category)
  TC  5–8  : Surface Position
  TC  9–18 : Airborne Position — barometric altitude + CPR lat/lon
  TC 19    : Airborne Velocity — ground speed or airspeed + heading + V/S
  TC 20–22 : Airborne Position — GNSS altitude + CPR lat/lon
  TC 28    : Aircraft Status
  TC 31    : Operational Status
"""

import argparse
import math
import socket
import struct
import time
from datetime import datetime, timezone
from typing import Optional

from pyModeS.message import crc_remainder as _crc_remainder  # table-driven CRC-24


# ═══════════════════════════════════════════════════════════════════════════════
# Section 1 — Low-level bit utilities
# ═══════════════════════════════════════════════════════════════════════════════

def get_bits(msg: str, start: int, end: int) -> int:
    """
    Extract bits [start .. end] from a hex-encoded Mode S message.

    Bit numbering is 1-indexed from the MSB of the entire message,
    matching the ICAO Annex 10 / DO-260B convention.

    Example: get_bits('8D4840D6...', 1, 5) → DF field.
    """
    n     = int(msg, 16)
    total = len(msg) * 4          # total bits in the hex string
    shift = total - end           # shift right so bit `end` becomes bit 0
    mask  = (1 << (end - start + 1)) - 1
    return (n >> shift) & mask


def me_payload(msg: str) -> int:
    """
    Return the 56-bit ME (Message Extended Squitter) payload as an integer.
    ME occupies bits 33–88 of the 112-bit message.
    """
    return get_bits(msg, 33, 88)


# ═══════════════════════════════════════════════════════════════════════════════
# Section 2 — CRC-24 (Mode S)
# ═══════════════════════════════════════════════════════════════════════════════
#
# pyModeS ships a table-driven CRC-24 implementation (~4.5x faster than a
# hand-rolled shift register).  crc_remainder(n, bits) returns 0 for a valid
# DF17/18 frame — the parity tail is included in the computation, so a zero
# remainder means the entire 112-bit message is intact.

def crc_valid(msg: str) -> bool:
    """Return True if the 112-bit message passes the Mode S CRC-24 check."""
    return _crc_remainder(int(msg, 16), 112) == 0


# ═══════════════════════════════════════════════════════════════════════════════
# Section 3 — Frame header parsing
# ═══════════════════════════════════════════════════════════════════════════════

def parse_header(msg: str) -> tuple:
    """
    Return (DF, ICAO, TC) for a raw 28-char hex message.

    DF   — Downlink Format, bits 1-5.   Should be 17 (or 18) for ADS-B.
    ICAO — 24-bit aircraft address, bits 9-32 (6 hex chars, upper-case).
    TC   — Type Code, first 5 bits of the ME payload (bits 33-37).
    """
    df   = get_bits(msg, 1, 5)
    icao = msg[2:8].upper()        # bits 9-32 map directly to hex chars 2-7
    tc   = get_bits(msg, 33, 37)
    return df, icao, tc


def tc_label(tc: int) -> str:
    """Human-readable category name for a Type Code."""
    if 1  <= tc <= 4:  return "Aircraft Identification"
    if 5  <= tc <= 8:  return "Surface Position"
    if 9  <= tc <= 18: return "Airborne Position (Baro)"
    if tc == 19:       return "Airborne Velocity"
    if 20 <= tc <= 22: return "Airborne Position (GNSS)"
    if tc == 28:       return "Aircraft Status"
    if tc == 31:       return "Operational Status"
    return f"Reserved (TC{tc})"


# ═══════════════════════════════════════════════════════════════════════════════
# Section 4 — Aircraft Identification (TC 1–4)
# ═══════════════════════════════════════════════════════════════════════════════
#
# ME layout (56 bits, 0-indexed from MSB of ME):
#   bits 0-4  : TC
#   bits 5-7  : Aircraft category (wake vortex)
#   bits 8-55 : 8 callsign characters x 6 bits each

# 64-character ACS charset: 6-bit index → ASCII character.
_CHARSET = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"

_WAKE_VORTEX = {
    0: "No category info",
    1: "Light  (< 7 500 kg)",
    2: "Medium 1  (7 500 – 34 000 kg)",
    3: "Medium 2  (34 000 – 136 000 kg)",
    4: "High vortex",
    5: "Heavy  (> 136 000 kg)",
    6: "High performance / high speed",
    7: "Rotorcraft",
}


def decode_identification(msg: str) -> dict:
    """
    Decode an Aircraft Identification message (TC 1–4).

    Category occupies ME bits 5-7  → message bits 38-40.
    Callsign: 8 chars x 6 bits, starting at ME bit 8 → message bit 41.
    """
    category = get_bits(msg, 38, 40)
    chars = []
    for i in range(8):
        start = 41 + i * 6
        chars.append(_CHARSET[get_bits(msg, start, start + 5)])
    callsign = "".join(chars).rstrip("#").strip()
    return {
        "callsign": callsign,
        "category": category,
        "wake":     _WAKE_VORTEX.get(category, "Unknown"),
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Section 5 — Altitude decoding
# ═══════════════════════════════════════════════════════════════════════════════
#
# TC 9-18 carries a 12-bit AC field at ME bits 8-19 (message bits 41-52).
# A zero M-bit is inserted at position 6 to form a 13-bit altcode:
#   altcode = ((ac >> 6) << 7) | (ac & 0x3F)
#
# 13-bit altcode layout (positions 0-12, MSB first):
#   C1 A1 C2 A2 C4 A4 M B1 Q B2 D2 B4 D4
#    0  1  2  3  4  5  6  7  8  9 10 11 12
#
# M=0, Q=1 → 25-ft linear:   altitude = 25 * N – 1000 ft
# M=0, Q=0 → Gillham code:   100-ft resolution, Gray-code variant

def _gray2int(n: int) -> int:
    """Convert a Gillham (reflected Gray) code to a plain binary integer."""
    n ^= n >> 8
    n ^= n >> 4
    n ^= n >> 2
    n ^= n >> 1
    return n


def _altcode_to_feet(ac: int) -> Optional[int]:
    """
    Convert a 12-bit AC field (from ADS-B position ME) to feet.
    Returns None for 'altitude unknown' or invalid Gillham codes.
    """
    if ac == 0:
        return None

    # Insert M=0 at position 6 to build the 13-bit altcode
    altcode = ((ac >> 6) << 7) | (ac & 0x3F)

    m_bit = (altcode >> 6) & 1   # bit 6 of the 13-bit field
    q_bit = (altcode >> 4) & 1   # bit 8 of the 13-bit field

    if m_bit == 0 and q_bit == 1:
        # 25-foot linear: remove M (bit 6) and Q (bit 4), form 11-bit N
        n = ((altcode >> 2) & 0x7E0) | ((altcode >> 1) & 0x10) | (altcode & 0xF)
        return n * 25 - 1000

    if m_bit == 0 and q_bit == 0:
        # Gillham (100-ft resolution) — extract named bits from altcode
        def b(pos: int) -> int:
            return (altcode >> (12 - pos)) & 1

        c1, a1 = b(0), b(1)
        c2, a2 = b(2), b(3)
        c4, a4 = b(4), b(5)
        b1      = b(7)
        b2, d2  = b(9), b(10)
        b4, d4  = b(11), b(12)

        gc500 = (d2<<7)|(d4<<6)|(a1<<5)|(a2<<4)|(a4<<3)|(b1<<2)|(b2<<1)|b4
        gc100 = (c1<<2)|(c2<<1)|c4

        n500 = _gray2int(gc500)
        n100 = _gray2int(gc100)

        if n100 in (0, 5, 6):
            return None              # reserved — invalid
        if n100 == 7:
            n100 = 5                 # remap per ICAO Annex 10 Vol IV
        if n500 % 2:
            n100 = 6 - n100          # odd 500-ft counter inverts 100-ft sense

        return n500 * 500 + n100 * 100 - 1300

    return None   # M=1: metric encoding, not common in ADS-B


def decode_altitude(msg: str) -> Optional[int]:
    """
    Extract and decode altitude from an airborne position message.
    TC 9-18  → barometric altitude (feet).
    TC 20-22 → GNSS altitude, 12-bit integer metres → feet.
    """
    tc = get_bits(msg, 33, 37)
    ac = get_bits(msg, 41, 52)    # ME bits 8-19 (12 bits)
    if 9  <= tc <= 18: return _altcode_to_feet(ac)
    if 20 <= tc <= 22: return int(ac * 3.28084)
    return None


# ═══════════════════════════════════════════════════════════════════════════════
# Section 6 — CPR (Compact Position Reporting) position decoding
# ═══════════════════════════════════════════════════════════════════════════════
#
# CPR encodes lat/lon as a 17-bit fractional offset within a lat/lon zone.
# A single frame is ambiguous — an even/odd pair resolves it.
#
# ME layout for airborne position (0-indexed from MSB of 56-bit ME):
#   bit  21 : F   — CPR format: 0 = even, 1 = odd   → message bit 54
#   bits 22-38 : CPR latitude  (17 bits)             → message bits 55-71
#   bits 39-55 : CPR longitude (17 bits)             → message bits 72-88
#
# Even frame uses 60 latitude zones (Dlat = 360/60 = 6°).
# Odd  frame uses 59 latitude zones (Dlat = 360/59 ≈ 6.1°).
# The coprime pair (59,60) uniquely resolves the zone via the CRT.

_NZ = 15  # ICAO CPR zone parameter (gives 4*NZ = 60 even zones)

def _NL(lat: float) -> int:
    """
    Longitude zone count at a given latitude.
    Returns 1 at the poles so longitude arithmetic stays well-defined.
    """
    if abs(lat) >= 87.0:
        return 1
    return int(
        2 * math.pi
        / math.acos(
            1.0 - (1.0 - math.cos(math.pi / (2 * _NZ)))
            / (math.cos(math.radians(lat)) ** 2)
        )
    )


def decode_cpr_fields(msg: str) -> tuple:
    """Return (cpr_format, cpr_lat_raw, cpr_lon_raw) from a position message."""
    return (
        get_bits(msg, 54, 54),   # F bit
        get_bits(msg, 55, 71),   # CPR lat (17 bits)
        get_bits(msg, 72, 88),   # CPR lon (17 bits)
    )


def cpr_resolve(
    cpr_lat_even: int, cpr_lon_even: int,
    cpr_lat_odd:  int, cpr_lon_odd:  int,
    use_odd: bool = False,
) -> Optional[tuple]:
    """
    Decode airborne position from a CPR even/odd pair.

    How it works
    ------------
    Each frame encodes the fractional offset within a latitude zone grid,
    but not which zone.  Two grids with coprime sizes (60 and 59) share only
    one consistent zone index j across all global latitudes — the Chinese
    Remainder Theorem guarantees uniqueness.

        j = floor(59 * lat_e_frac - 60 * lat_o_frac + 0.5)

        lat_even = (360/60) * (j mod 60 + lat_e_frac)
        lat_odd  = (360/59) * (j mod 59 + lat_o_frac)

    The same approach resolves longitude using NL(lat)-based zone counts.

    Parameters
    ----------
    use_odd : if True, report the position of the odd frame (default: even).

    Returns
    -------
    (latitude°, longitude°) or None if frames straddle a zone boundary.
    """
    lat_e = cpr_lat_even / 131072.0
    lon_e = cpr_lon_even / 131072.0
    lat_o = cpr_lat_odd  / 131072.0
    lon_o = cpr_lon_odd  / 131072.0

    j = math.floor(59.0 * lat_e - 60.0 * lat_o + 0.5)

    lat_even = (360.0 / 60.0) * (j % 60 + lat_e)
    lat_odd  = (360.0 / 59.0) * (j % 59 + lat_o)

    if lat_even >= 270.0: lat_even -= 360.0
    if lat_odd  >= 270.0: lat_odd  -= 360.0

    # Zone-boundary consistency check
    if _NL(lat_even) != _NL(lat_odd):
        return None

    lat = lat_odd if use_odd else lat_even
    nl  = _NL(lat)
    ni  = max(nl - (1 if use_odd else 0), 1)

    m   = math.floor(lon_e * (nl - 1) - lon_o * nl + 0.5)
    lon = (360.0 / ni) * (m % ni + (lon_o if use_odd else lon_e))

    if lon >= 180.0: lon -= 360.0

    return lat, lon


# ═══════════════════════════════════════════════════════════════════════════════
# Section 7 — Airborne Velocity (TC 19)
# ═══════════════════════════════════════════════════════════════════════════════
#
# ME layout (0-indexed from MSB of 56-bit ME):
#   bits 5-7:   Subtype (1-4)
#   bits 10-12: NAC_v
#
#   Subtypes 1/2 — Ground speed (scale x1 or x4):
#     bit  13 : v_ew direction  (0=east, 1=west)
#     bits 14-23 : v_ew magnitude  (actual = value - 1 kt)
#     bit  24 : v_ns direction  (0=north, 1=south)
#     bits 25-34 : v_ns magnitude  (actual = value - 1 kt)
#
#   Subtypes 3/4 — Airspeed (scale x1 or x4):
#     bit  13 : heading status  (1=valid)
#     bits 14-23 : heading raw  (actual = raw * 360/1024 deg)
#     bit  24 : airspeed type   (0=IAS, 1=TAS)
#     bits 25-34 : airspeed     (actual = value - 1 kt)
#
#   Common trailer:
#     bit  35 : VR source   (0=GNSS, 1=BARO)
#     bit  36 : VR sign     (0=climb, 1=descent)
#     bits 37-45 : VR mag   (actual = (value - 1) * 64 ft/min)
#     bit  48 : GNSS-baro sign
#     bits 49-55 : GNSS-baro mag  (actual = (value - 1) * 25 ft)

def decode_velocity(msg: str) -> dict:
    """Decode an Airborne Velocity message (TC 19)."""
    p = me_payload(msg)

    subtype = (p >> 48) & 0x7
    nac_v   = (p >> 43) & 0x7
    result  = {"subtype": subtype, "nac_v": nac_v}

    if subtype in (1, 2):
        scale     = 4 if subtype == 2 else 1
        v_ew_sign = (p >> 42) & 1
        v_ew_mag  = (p >> 32) & 0x3FF
        v_ns_sign = (p >> 31) & 1
        v_ns_mag  = (p >> 21) & 0x3FF

        if v_ew_mag and v_ns_mag:
            v_ew  = (v_ew_mag - 1) * scale
            v_ns  = (v_ns_mag - 1) * scale
            v_we  = -v_ew if v_ew_sign else v_ew
            v_sn  = -v_ns if v_ns_sign else v_ns
            speed = int(math.hypot(v_we, v_sn))
            track = math.degrees(math.atan2(v_we, v_sn)) % 360.0
            result.update({"speed": speed, "track": track, "speed_type": "GS"})
        else:
            result.update({"speed": None, "track": None, "speed_type": "GS"})

    elif subtype in (3, 4):
        scale      = 4 if subtype == 4 else 1
        hdg_status = (p >> 42) & 1
        hdg_raw    = (p >> 32) & 0x3FF
        as_type    = (p >> 31) & 1
        as_mag     = (p >> 21) & 0x3FF

        result.update({
            "speed":      None if as_mag == 0 else (as_mag - 1) * scale,
            "heading":    (hdg_raw / 1024.0 * 360.0) if hdg_status else None,
            "speed_type": "TAS" if as_type else "IAS",
        })

    # Vertical rate (common to all subtypes)
    vr_src = (p >> 20) & 1
    vr_sgn = (p >> 19) & 1
    vr_mag = (p >> 10) & 0x1FF
    result["vr_source"]     = "BARO" if vr_src else "GNSS"
    result["vertical_rate"] = (
        None if vr_mag == 0
        else (-1 if vr_sgn else 1) * (vr_mag - 1) * 64
    )

    # GNSS-minus-barometric altitude difference
    diff_sign = (p >> 7) & 1
    diff_mag  = p & 0x7F
    result["geo_minus_baro"] = (
        None if diff_mag in (0, 127)
        else (-1 if diff_sign else 1) * (diff_mag - 1) * 25
    )

    return result


# ═══════════════════════════════════════════════════════════════════════════════
# Section 8 — Aircraft state tracker
# ═══════════════════════════════════════════════════════════════════════════════

class Aircraft:
    """
    Aggregates decoded ADS-B state for one ICAO address.

    CPR position is resolved on each ingest_cpr() call once a valid even/odd
    pair has been seen within a 10-second window (per DO-260B §2.2.3.2.7.1).
    """

    __slots__ = (
        "icao", "callsign", "category", "wake",
        "altitude", "lat", "lon",
        "speed", "heading", "track", "vrate",
        "speed_type", "vr_source",
        "last_seen", "_cpr_even", "_cpr_odd",
    )

    def __init__(self, icao: str) -> None:
        self.icao       = icao.upper()
        self.callsign   = None
        self.category   = None
        self.wake       = None
        self.altitude   = None
        self.lat        = None
        self.lon        = None
        self.speed      = None
        self.heading    = None
        self.track      = None
        self.vrate      = None
        self.speed_type = None
        self.vr_source  = None
        self.last_seen  = None
        self._cpr_even  = None
        self._cpr_odd   = None

    def ingest_cpr(self, fmt: int, cpr_lat: int, cpr_lon: int,
                   alt: Optional[int]) -> None:
        """Store a CPR frame and resolve position when a valid pair exists."""
        frame = {"lat": cpr_lat, "lon": cpr_lon, "alt": alt, "ts": time.monotonic()}
        if fmt == 0:
            self._cpr_even = frame
        else:
            self._cpr_odd = frame

        if self._cpr_even and self._cpr_odd:
            age = abs(self._cpr_even["ts"] - self._cpr_odd["ts"])
            if age <= 10.0:
                pos = cpr_resolve(
                    self._cpr_even["lat"], self._cpr_even["lon"],
                    self._cpr_odd["lat"],  self._cpr_odd["lon"],
                    use_odd=(fmt == 1),
                )
                if pos:
                    self.lat, self.lon = pos
                    if alt is not None:
                        self.altitude = alt

    def summary(self) -> str:
        """One-line summary of all known state for this aircraft."""
        parts = [f"ICAO: {self.icao}"]
        if self.callsign:
            parts.append(f"Callsign: {self.callsign}")
        if self.wake:
            parts.append(f"Wake: {self.wake}")
        if self.altitude is not None:
            parts.append(f"Alt: {self.altitude:,} ft")
        if self.lat is not None:
            parts.append(f"Pos: ({self.lat:.4f}°, {self.lon:.4f}°)")
        if self.speed is not None:
            parts.append(f"Speed: {self.speed} kt [{self.speed_type}]")
        if self.track is not None:
            parts.append(f"Track: {self.track:.1f}°")
        if self.heading is not None:
            parts.append(f"Hdg: {self.heading:.1f}°")
        if self.vrate is not None:
            arrow = "▲" if self.vrate > 0 else ("▼" if self.vrate < 0 else "—")
            parts.append(f"V/S: {arrow}{abs(self.vrate):,} fpm [{self.vr_source}]")
        return "  │  ".join(parts)


# ═══════════════════════════════════════════════════════════════════════════════
# Section 9 — Top-level message decoder
# ═══════════════════════════════════════════════════════════════════════════════

def decode_message(raw: str, fleet: dict) -> dict:
    """
    Decode a single raw ADS-B hex string and update the fleet state dict.

    raw   — 28-char hex string (dump1090 '*HEX;' framing is stripped).
    fleet — dict[icao → Aircraft], updated in-place.

    Returns a dict with at minimum: raw, valid (bool), summary (str).
    Failed decodes also carry an 'error' key.
    """
    raw = raw.strip().upper().lstrip("*").rstrip(";")

    result = {"raw": raw, "valid": False, "summary": ""}

    if len(raw) != 28:
        result["error"] = f"Expected 28 hex chars, got {len(raw)}"
        return result

    try:
        int(raw, 16)
    except ValueError:
        result["error"] = "Non-hex characters in message"
        return result

    if not crc_valid(raw):
        result["error"] = "CRC-24 invalid"
        return result

    df, icao, tc = parse_header(raw)
    result.update({"df": df, "icao": icao, "tc": tc, "tc_label": tc_label(tc)})

    if df not in (17, 18):
        result["valid"]   = True
        result["summary"] = f"DF{df} — not an ADS-B ES frame"
        return result

    if icao not in fleet:
        fleet[icao] = Aircraft(icao)
    ac = fleet[icao]
    ac.last_seen = datetime.now(timezone.utc)
    result["valid"] = True

    # ── TC 1–4 : Aircraft Identification ────────────────────────────────────
    if 1 <= tc <= 4:
        ident       = decode_identification(raw)
        ac.callsign = ident["callsign"]
        ac.category = ident["category"]
        ac.wake     = ident["wake"]
        result.update(ident)
        result["summary"] = (
            f"[IDENT]  {icao}  "
            f"Callsign: {ac.callsign}  Wake: {ac.wake}"
        )

    # ── TC 9–18, 20–22 : Airborne Position ──────────────────────────────────
    elif (9 <= tc <= 18) or (20 <= tc <= 22):
        fmt, cpr_lat, cpr_lon = decode_cpr_fields(raw)
        alt = decode_altitude(raw)
        result.update({"cpr_format": fmt, "cpr_lat": cpr_lat,
                        "cpr_lon": cpr_lon, "altitude": alt})
        ac.ingest_cpr(fmt, cpr_lat, cpr_lon, alt)
        pos = f"({ac.lat:.4f}°, {ac.lon:.4f}°)" if ac.lat is not None else "awaiting pair"
        alt_str = f"{alt:,} ft" if alt is not None else "unknown"
        result["summary"] = (
            f"[POS]    {icao}  "
            f"Alt: {alt_str}  "
            f"CPR-{'even' if fmt == 0 else 'odd'}  Pos: {pos}"
        )

    # ── TC 19 : Airborne Velocity ────────────────────────────────────────────
    elif tc == 19:
        vel           = decode_velocity(raw)
        ac.speed      = vel.get("speed")
        ac.heading    = vel.get("heading")
        ac.track      = vel.get("track")
        ac.vrate      = vel.get("vertical_rate")
        ac.speed_type = vel.get("speed_type")
        ac.vr_source  = vel.get("vr_source")
        result.update(vel)

        direction = ac.track if ac.track is not None else ac.heading
        vr        = ac.vrate or 0
        arrow     = "▲" if vr > 0 else ("▼" if vr < 0 else "—")
        dir_str   = f"{direction:.1f}°" if direction is not None else "unknown"
        result["summary"] = (
            f"[VEL]    {icao}  "
            f"Speed: {ac.speed} kt [{ac.speed_type}]  "
            f"Dir: {dir_str}  "
            f"V/S: {arrow}{abs(vr):,} fpm [{ac.vr_source}]"
        )

    # ── TC 28/31 : Status / Operational ─────────────────────────────────────
    elif tc in (28, 31):
        result["summary"] = f"[{'STAT' if tc==28 else 'OPS'}]    {icao}  {tc_label(tc)}"

    else:
        result["summary"] = f"[TC{tc:02d}]    {icao}  {tc_label(tc)}"

    return result


# ═══════════════════════════════════════════════════════════════════════════════
# Section 10 — CLI / I/O helpers
# ═══════════════════════════════════════════════════════════════════════════════

_BANNER = (
    "\n╔══════════════════════════════════════════════════════════════════╗\n"
    "║              ADS-B DECODER — pure Python stdlib                 ║\n"
    "╚══════════════════════════════════════════════════════════════════╝\n"
)
_SEP = "─" * 68

_DEMO = [
    # TC 4 — identification, KLM1023
    "8D4840D6202CC371C32CE0576098",
    # TC 11 — airborne position, even frame, 38 000 ft
    "8D40621D58C382D690C8AC2863A7",
    # TC 11 — airborne position, odd frame (enables CPR decode)
    "8D40621D58C386435CC412692AD6",
    # TC 19 subtype 3 — TAS 375 kt, hdg 244°, −2 304 ft/min
    "8DA05F219B06B6AF189400CBC33F",
]


def _ts() -> str:
    return datetime.now(timezone.utc).strftime("%H:%M:%S")

def _print_result(res: dict) -> None:
    raw = res.get("raw", "")
    if res.get("valid"):
        print(f"  [{_ts()}]  {raw}  →  {res['summary']}")
    else:
        print(f"  [{_ts()}]  {raw}  ✗  {res.get('error', '?')}")

def _print_fleet(fleet: dict) -> None:
    if not fleet:
        return
    print(f"\n{'═'*68}\n  FLEET STATE\n{'═'*68}")
    for ac in fleet.values():
        print(f"  {ac.summary()}")
    print(f"{'═'*68}\n")


def run_demo(fleet: dict) -> None:
    print(_BANNER)
    print("  Demo messages\n" + _SEP)
    for raw in _DEMO:
        _print_result(decode_message(raw, fleet))
    _print_fleet(fleet)


def run_single(raw: str, fleet: dict) -> None:
    res = decode_message(raw, fleet)
    _print_result(res)
    if res["valid"]:
        print()
        for k, v in res.items():
            if k not in ("raw", "valid", "summary"):
                print(f"  {k:<22}: {v}")
    _print_fleet(fleet)


def run_file(path: str, fleet: dict) -> None:
    print(_BANNER)
    print(f"  File: {path}\n" + _SEP)
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            _print_result(decode_message(line, fleet))
    _print_fleet(fleet)


def run_live(host: str, port: int, fleet: dict) -> None:
    """Connect to dump1090 raw TCP port and decode messages in real time."""
    print(_BANNER)
    print(f"  Connecting to dump1090 at {host}:{port}\n" + _SEP)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        buf = ""
        while True:
            chunk = s.recv(4096).decode("ascii", errors="ignore")
            buf  += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    res = decode_message(line, fleet)
                    if res.get("valid"):
                        _print_result(res)


def _print_fleet_table(fleet: dict, source: str) -> None:
    """Clear the terminal and print a tidy coordinate table for all tracked aircraft."""
    now = datetime.now(timezone.utc)
    print("\033[2J\033[H", end="")
    hdr = f"{'ICAO':<8}{'Callsign':<10}{'Latitude':>12}{'Longitude':>13}{'Alt ft':>9}{'Spd kt':>8}{'Track°':>8}{'Seen':>8}"
    print(hdr)
    print("─" * len(hdr))
    stale_cutoff = 30
    shown = 0
    for ac in sorted(fleet.values(), key=lambda a: a.icao):
        age = (now - ac.last_seen).total_seconds() if ac.last_seen else 9999
        if age > stale_cutoff:
            continue
        lat = f"{ac.lat:+.5f}°"   if ac.lat      is not None else "—"
        lon = f"{ac.lon:+.5f}°"   if ac.lon      is not None else "—"
        alt = f"{ac.altitude:,}"  if ac.altitude  is not None else "—"
        spd = str(ac.speed)       if ac.speed     is not None else "—"
        trk = (f"{ac.track:.1f}"   if ac.track    is not None else
               f"{ac.heading:.1f}" if ac.heading  is not None else "—")
        cs  = ac.callsign or "—"
        print(f"{ac.icao:<8}{cs:<10}{lat:>12}{lon:>13}{alt:>9}{spd:>8}{trk:>8}{age:>7.0f}s")
        shown += 1
    if shown == 0:
        print("  (no aircraft — waiting for messages)")
    print(f"\n  {_ts()} UTC   {source}   {len(fleet)} tracked")


def run_multicast(group: str, port: int, iface: str, fleet: dict) -> None:
    """Join a UDP multicast group and print a live coordinate table."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))

    mreq = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton(iface))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(1.0)

    buf = ""
    last_print = 0.0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                buf += data.decode("ascii", errors="ignore")
            except socket.timeout:
                pass
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    decode_message(line, fleet)
            now = time.monotonic()
            if now - last_print >= 1.0:
                _print_fleet_table(fleet, f"{group}:{port}")
                last_print = now
    finally:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
        sock.close()


# ═══════════════════════════════════════════════════════════════════════════════
# Section 11 — Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="ADS-B decoder — pure Python stdlib",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    g = parser.add_mutually_exclusive_group()
    g.add_argument("--msg",  metavar="HEX",  help="Decode one 28-char hex message")
    g.add_argument("--file", metavar="PATH", help="Decode a file of hex messages")
    g.add_argument("--live", action="store_true",
                   help="Live decode from dump1090 TCP stream")
    g.add_argument("--multicast", action="store_true",
                   help="Live decode from UDP multicast stream (default port 30003)")
    parser.add_argument("--host",  default="127.0.0.1",
                        help="TCP host for --live (default: 127.0.0.1)")
    parser.add_argument("--port",  type=int, default=None,
                        help="Port: 30002 for --live, 30003 for --multicast")
    parser.add_argument("--group", default="239.255.0.1",
                        help="Multicast group address (default: 239.255.0.1)")
    parser.add_argument("--iface", default="0.0.0.0",
                        help="Local interface for multicast (default: 0.0.0.0)")
    args = parser.parse_args()

    fleet: dict = {}
    if args.msg:
        run_single(args.msg, fleet)
    elif args.file:
        run_file(args.file, fleet)
    elif args.live:
        run_live(args.host, args.port if args.port is not None else 30002, fleet)
    elif args.multicast:
        run_multicast(args.group, args.port if args.port is not None else 30003,
                      args.iface, fleet)
    else:
        run_demo(fleet)


if __name__ == "__main__":
    main()
