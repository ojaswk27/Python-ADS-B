"""
IFF reply binary format (Mark-XIIA-style placeholder).

Header (10 B, big-endian):
    uint32  magic        = 0xAABB0102   (placeholder)
    uint16  prt_no
    uint16  az_counter   = int(azimuth_deg * 4096 / 360) & 0xFFF
    uint8   mode         (see MODE_* below)
    uint8   n_targets

Per-target record (big-endian), one of:

  Modes 1 / 2 / 3-A / C  → 48 B
    uint32  range_counter   (LSB = RANGE_LSB_NM nautical miles)
    uint16  code            (12-bit octal squawk in low 12 bits, or Mode C alt)
    uint16  confidence      (placeholder 0xFFFF = "all bits confident")
    40 B    spare           (zero)

  Mode S (All-Call + Selective)  → 50 B
    uint32  range_counter
    14 B    mode_s_data       (byte 0-2 = ICAO 24-bit; rest = 0 placeholder)
    14 B    mode_s_confidence (placeholder 0xFF * 14)
    18 B    spare             (zero)

Mode C altitude packing into the 16-bit Code field: sign at bit 11, magnitude
in bits 0-10 = |altitude_ft| / 25, capped at 2047.
"""

import struct
from dataclasses import dataclass

# ── Constants ─────────────────────────────────────────────────────────────────

HEADER_MAGIC = 0xAABB0102

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

RANGE_LSB_NM = 1.0 / 128.0
MAX_TARGETS  = 20

_RECORD_LEN_CLASSIC = 48
_RECORD_LEN_MODE_S  = 50
_HEADER_LEN         = 10


# ── Target record ─────────────────────────────────────────────────────────────

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


# ── Encoders ──────────────────────────────────────────────────────────────────

def encode_mode_c(altitude_ft: int) -> int:
    """Pack altitude into the 16-bit Code field for Mode C replies."""
    sign = 0 if altitude_ft >= 0 else 1
    mag  = min(abs(int(altitude_ft)) // 25, 0x7FF)        # 11 bits
    return (sign << 11) | mag


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


# ── Decoder (for the reply log) ───────────────────────────────────────────────

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
