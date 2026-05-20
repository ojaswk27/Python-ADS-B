#!/usr/bin/env python3
"""
ADS-B Aircraft Emulator
=======================
Simulates aircraft transmitting ADS-B messages over UDP multicast (port 30003).
Messages use the raw hex framing that dump1090 emits (*HEXMSG;\\n).

Usage
-----
    python aircraft_emulator.py                       # 3 aircraft → 239.255.0.1:30003
    python aircraft_emulator.py --count 2             # fewer aircraft
    python aircraft_emulator.py --port 30003          # explicit port
    python aircraft_emulator.py --group 239.255.0.1 --iface 127.0.0.1
    python aircraft_emulator.py --rate 5              # messages/second per aircraft
"""

import argparse
import math
import socket
import struct
import time
from dataclasses import dataclass, field

from pyModeS.message import crc_remainder as _crc_remainder
import net_config


# ─── CPR helpers ─────────────────────────────────────────────────────────────

_NZ = 15


def _nl(lat: float) -> int:
    """Longitude zone count for CPR at a given latitude."""
    if abs(lat) >= 87.0:
        return 1
    return int(
        2 * math.pi
        / math.acos(
            1.0 - (1.0 - math.cos(math.pi / (2 * _NZ)))
            / math.cos(math.radians(lat)) ** 2
        )
    )


def _encode_cpr(lat: float, lon: float, odd: bool) -> tuple:
    """
    Encode a WGS-84 position to a 17-bit CPR (cpr_lat, cpr_lon) pair.

    The encoding uses the same zone-grid arithmetic as the decoder's cpr_resolve,
    but in reverse: divide the lat/lon fractional offset within its zone by the
    zone width and scale to 2^17.  odd=True selects the 59-zone (odd) grid.
    """
    dlat = 360.0 / (4 * _NZ - (1 if odd else 0))
    rlat = lat % dlat
    if rlat < 0:
        rlat += dlat
    cpr_lat = int(131072.0 * rlat / dlat + 0.5) % 131072

    nl = _nl(lat)
    ni = max(nl - (1 if odd else 0), 1)
    dlon = 360.0 / ni
    rlon = lon % dlon
    if rlon < 0:
        rlon += dlon
    cpr_lon = int(131072.0 * rlon / dlon + 0.5) % 131072

    return cpr_lat, cpr_lon


# ─── Altitude encoding (Q=1, 25-ft linear) ────────────────────────────────────

def _encode_altitude(alt_ft: int) -> int:
    """
    Return the 12-bit AC field for the given altitude.

    Uses the Q=1 (25-ft linear) encoding: the 13-bit altcode has M=0 at bit 6
    and Q=1 at bit 4; the AC field is the 12-bit result after stripping M.
    Valid range: −1 000 ft to roughly 50 000 ft.
    """
    n = max(0, (alt_ft + 1000) // 25)
    # Build 13-bit altcode with M=0 at bit 6 and Q=1 at bit 4
    altcode = ((n >> 5) << 7) | (((n >> 4) & 1) << 5) | (1 << 4) | (n & 0xF)
    # Strip M bit to recover 12-bit AC field
    return ((altcode >> 7) << 6) | (altcode & 0x3F)


# ─── Callsign encoding (ACS 6-bit charset) ────────────────────────────────────

_CHARSET = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"


def _char_idx(c: str) -> int:
    i = _CHARSET.find(c)
    return i if i >= 0 else 0


# ─── Message builders ─────────────────────────────────────────────────────────

def _sign_crc(payload_hex: str) -> str:
    """Given an 11-byte (22-char) hex payload, return the full 14-byte message."""
    msg0 = payload_hex + "000000"
    crc_val = _crc_remainder(int(msg0, 16), 112)
    return payload_hex + f"{crc_val:06X}"


def build_identification(icao: str, callsign: str, category: int = 3) -> str:
    """TC=4 Aircraft Identification message."""
    icao_int = int(icao, 16)
    tc = 4
    me = (tc << 51) | (category << 48)
    padded = (callsign.upper() + "        ")[:8]
    for i, ch in enumerate(padded):
        me |= _char_idx(ch) << (42 - i * 6)
    header = (0x8D << 80) | (icao_int << 56) | me
    return _sign_crc(f"{header:022X}")


def build_position(icao: str, lat: float, lon: float,
                   alt_ft: int, odd: bool) -> str:
    """TC=11 Airborne Position message (barometric altitude, CPR encoded)."""
    icao_int = int(icao, 16)
    tc = 11
    ac = _encode_altitude(alt_ft)
    fmt = 1 if odd else 0
    cpr_lat, cpr_lon = _encode_cpr(lat, lon, odd)
    # ME: TC(5)|SS(2)|NIC_B(1)|ALT(12)|T(1)|F(1)|CPR_LAT(17)|CPR_LON(17)
    me = (tc << 51) | (ac << 36) | (fmt << 34) | (cpr_lat << 17) | cpr_lon
    header = (0x8D << 80) | (icao_int << 56) | me
    return _sign_crc(f"{header:022X}")


def build_velocity(icao: str, speed_kt: float, heading_deg: float,
                   vrate_fpm: int) -> str:
    """TC=19 subtype-1 Airborne Velocity (ground speed) message."""
    icao_int = int(icao, 16)
    tc = 19
    subtype = 1

    hdg = math.radians(heading_deg)
    v_ew = speed_kt * math.sin(hdg)   # positive = east
    v_ns = speed_kt * math.cos(hdg)   # positive = north

    ew_sign = 1 if v_ew < 0 else 0
    ns_sign = 1 if v_ns < 0 else 0
    ew_mag  = min(int(abs(v_ew)) + 1, 1023)
    ns_mag  = min(int(abs(v_ns)) + 1, 1023)

    vr_sign = 1 if vrate_fpm < 0 else 0
    vr_mag  = min(abs(vrate_fpm) // 64 + 1, 511)

    # ME: TC(5)|subtype(3)|intent(1)|IFR(1)|NAC_v(3)|
    #     ew_sign(1)|ew_mag(10)|ns_sign(1)|ns_mag(10)|
    #     vr_src(1)|vr_sign(1)|vr_mag(9)|rsvd(2)|diff_sign(1)|diff(7)
    me = ((tc << 51)
          | (subtype << 48)
          | (ew_sign << 42)
          | (ew_mag  << 32)
          | (ns_sign << 31)
          | (ns_mag  << 21)
          | (1       << 20)   # vr_src = BARO
          | (vr_sign << 19)
          | (vr_mag  << 10))
    header = (0x8D << 80) | (icao_int << 56) | me
    return _sign_crc(f"{header:022X}")


# ─── Simulated aircraft state ─────────────────────────────────────────────────

@dataclass
class SimAircraft:
    icao:        str
    callsign:    str
    category:    int
    lat:         float
    lon:         float
    alt_ft:      int
    speed_kt:    float
    heading_deg: float
    vrate_fpm:   int
    # heading change per second (for circling aircraft)
    turn_rate:   float = 0.0
    _msg_idx:    int = field(default=0, repr=False)

    KT_PER_DEG_LAT = 60.0          # 1° lat ≈ 60 nm

    def step(self, dt: float) -> None:
        """Advance position, altitude, and heading by dt seconds."""
        hdg = math.radians(self.heading_deg)
        nm  = self.speed_kt * dt / 3600.0   # nautical miles
        self.lat += nm * math.cos(hdg) / self.KT_PER_DEG_LAT
        self.lon += (nm * math.sin(hdg)
                     / (self.KT_PER_DEG_LAT * math.cos(math.radians(self.lat))))
        self.alt_ft   = max(0, self.alt_ft + int(self.vrate_fpm * dt / 60.0))
        self.heading_deg = (self.heading_deg + self.turn_rate * dt) % 360.0

    def next_message(self) -> tuple[str, str]:
        """
        Return (label, raw_hex) for the next message in the rotation.
        Rotation: ident → even pos → odd pos → velocity → even pos → odd pos → velocity …
        """
        seq = self._msg_idx % 6
        self._msg_idx += 1

        if seq == 0:
            return "IDENT", build_identification(self.icao, self.callsign, self.category)
        elif seq in (1, 4):
            return "POS-E", build_position(self.icao, self.lat, self.lon, self.alt_ft, False)
        elif seq in (2, 5):
            return "POS-O", build_position(self.icao, self.lat, self.lon, self.alt_ft, True)
        else:  # seq in (3,)
            return "VEL ", build_velocity(self.icao, self.speed_kt,
                                          self.heading_deg, self.vrate_fpm)


# ─── Default fleet ────────────────────────────────────────────────────────────

_ALL_AIRCRAFT = [
    SimAircraft(
        icao="4840D6", callsign="KLM1023", category=3,
        lat=52.0, lon=4.0, alt_ft=38000, speed_kt=480,
        heading_deg=45, vrate_fpm=0,
    ),
    SimAircraft(
        icao="A05F21", callsign="UAL456", category=3,
        lat=40.71, lon=-74.00, alt_ft=35000, speed_kt=520,
        heading_deg=270, vrate_fpm=-64,
    ),
    SimAircraft(
        icao="7C4516", callsign="QFA007", category=3,
        lat=-33.87, lon=151.21, alt_ft=12000, speed_kt=250,
        heading_deg=190, vrate_fpm=-256,
    ),
    SimAircraft(
        icao="3C6444", callsign="DLH400", category=3,
        lat=48.35, lon=11.78, alt_ft=5000, speed_kt=180,
        heading_deg=0, vrate_fpm=512,
        turn_rate=3.0,   # circling: 3°/s → full circle in 2 minutes
    ),
]


# ─── Emitter loop ─────────────────────────────────────────────────────────────

def run_emulator(aircraft: list, group: str, port: int,
                 iface: str, rate: float) -> None:
    """Send ADS-B messages for all aircraft over UDP multicast."""
    interval = 1.0 / rate          # seconds between messages per aircraft
    total_interval = interval / len(aircraft)  # overall dispatch interval

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                    socket.inet_aton(iface))

    print(f"[emulator] sending to {group}:{port}  iface={iface}  "
          f"{len(aircraft)} aircraft  {rate} msg/s per aircraft")
    for ac in aircraft:
        print(f"  ICAO={ac.icao}  callsign={ac.callsign}  "
              f"alt={ac.alt_ft:,} ft  hdg={ac.heading_deg:.0f}°  "
              f"spd={ac.speed_kt:.0f} kt")
    print()

    ac_idx = 0
    last_step = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            dt  = now - last_step
            last_step = now

            ac = aircraft[ac_idx % len(aircraft)]
            ac.step(dt * len(aircraft))   # each aircraft steps its full share

            label, raw = ac.next_message()
            line = f"*{raw};\n"
            sock.sendto(line.encode("ascii"), (group, port))

            print(f"  [{time.strftime('%H:%M:%S')}]  {ac.icao}  {label}  {raw}")

            ac_idx += 1
            time.sleep(total_interval)
    except KeyboardInterrupt:
        print("\n[emulator] stopped")
    finally:
        sock.close()


# ─── Entry point ─────────────────────────────────────────────────────────────

def main() -> None:
    _cfg = net_config.load()
    parser = argparse.ArgumentParser(
        description="ADS-B aircraft emulator — sends messages over UDP multicast",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--group",  default=_cfg["group"],
                        help=f"Multicast group (default: {_cfg['group']})")
    parser.add_argument("--port",   type=int, default=_cfg["port"],
                        help=f"UDP port (default: {_cfg['port']})")
    parser.add_argument("--iface",  default=_cfg["iface"],
                        help=f"Local interface (default: {_cfg['iface']})")
    parser.add_argument("--count",  type=int, default=3,
                        help="Number of aircraft to simulate, 1–4 (default: 3)")
    parser.add_argument("--rate",   type=float, default=2.0,
                        help="Messages per second per aircraft (default: 2)")
    args = parser.parse_args()

    count = max(1, min(args.count, len(_ALL_AIRCRAFT)))
    run_emulator(_ALL_AIRCRAFT[:count], args.group, args.port,
                 args.iface, args.rate)


if __name__ == "__main__":
    main()
