#!/usr/bin/env python3
"""
ADS-B PPI Display  ·  ASTERIX CAT021
=====================================
Radar-style Plan Position Indicator (PPI) fed from UDP multicast.
Decodes ADS-B messages in a continuous loop and renders a live sweep-radar
display.  The right-hand panel shows each track formatted as ASTERIX CAT021
Edition 2.1 data items.  Optionally writes binary ASTERIX records to a file.

Usage
-----
    python ppi_display.py                          # 239.255.0.1:30003, auto-centre
    python ppi_display.py --iface 127.0.0.1        # loopback (for emulator testing)
    python ppi_display.py --centre 52.0,4.0        # fix radar centre
    python ppi_display.py --range 150              # initial range in nm (default 300)
    python ppi_display.py --asterix out.ast        # also write binary ASTERIX records

Keys (while running)
--------------------
    + / =   zoom in (75 % range)
    -       zoom out (133 % range)
    q       quit
"""

import argparse
import curses
import math
import socket
import struct
import threading
import time
from datetime import datetime, timezone
from typing import Optional

from adsb_decoder import Aircraft, decode_message


# ═══════════════════════════════════════════════════════════════════════════════
# Section 1 — ASTERIX CAT021 Edition 2.1 binary encoder
# ═══════════════════════════════════════════════════════════════════════════════
#
# UAP (User Application Profile), Edition 2.1:
#   FSPEC byte 0  bits 7-1: FRN 1-7   (I021/010 040 030 130 080 140 090)
#   FSPEC byte 1  bits 7-1: FRN 8-14  (I021/210 070 230 145 150 151 152)
#   FSPEC byte 2  bits 7-1: FRN 15-21 (I021/155 157 160 165 170 095 032)
#   bit 0 of each FSPEC byte = FX (1 → more FSPEC bytes follow)
#
# Only items we have data for are encoded.

_SAC = 0x00   # System Area Code   (configurable in real installations)
_SIC = 0x01   # System Identification Code


def _frn_bit(frn: int) -> tuple:
    """Return (fspec_byte_index, bit_position) for a given FRN."""
    return (frn - 1) // 7, 7 - (frn - 1) % 7


def encode_cat021(ac: Aircraft) -> bytes:
    """
    Produce a complete ASTERIX data block (category + length + record)
    for the given Aircraft state.

    Items encoded when data is available:
        FRN  1  I021/010  Data Source Identifier (SAC/SIC)
        FRN  3  I021/030  Time of Day (1/128 s since midnight UTC)
        FRN  4  I021/130  Position in WGS-84 (LSB = 180/2^23 °, ≈0.4 m)
        FRN  5  I021/080  Target Address (24-bit ICAO)
        FRN  6  I021/140  Geometric Height (LSB = 6.25 ft)
        FRN 11  I021/145  Flight Level (LSB = ¼ FL = 25 ft)
        FRN 15  I021/155  Barometric Vertical Rate (LSB = 6.25 ft/min)
        FRN 19  I021/170  Track Angle (LSB = 360/2^16 °)
    """
    items: dict[int, bytes] = {}

    # FRN 1 — I021/010 Data Source Identifier (always present)
    items[1] = bytes([_SAC, _SIC])

    # FRN 3 — I021/030 Time of Day (3 bytes, 1/128 s since midnight)
    now  = datetime.now(timezone.utc)
    secs = now.hour * 3600 + now.minute * 60 + now.second + now.microsecond * 1e-6
    items[3] = (int(secs * 128) & 0xFFFFFF).to_bytes(3, "big")

    # FRN 4 — I021/130 Position (3+3 signed bytes, LSB = 180/2^23 °)
    if ac.lat is not None and ac.lon is not None:
        lsb   = 180.0 / (1 << 23)
        lat_i = max(-8_388_608, min(8_388_607, round(ac.lat / lsb)))
        lon_i = max(-8_388_608, min(8_388_607, round(ac.lon / lsb)))
        items[4] = (lat_i.to_bytes(3, "big", signed=True)
                    + lon_i.to_bytes(3, "big", signed=True))

    # FRN 5 — I021/080 Target Address (24-bit ICAO)
    if ac.icao:
        items[5] = bytes.fromhex(ac.icao.zfill(6))

    # FRN 6 — I021/140 Geometric Height (signed 16-bit, LSB = 6.25 ft)
    if ac.altitude is not None:
        h = max(-32_768, min(32_767, round(ac.altitude / 6.25)))
        items[6] = h.to_bytes(2, "big", signed=True)

    # FRN 11 — I021/145 Flight Level (signed 16-bit, LSB = 1/4 FL = 25 ft)
    if ac.altitude is not None:
        fl = max(-32_768, min(32_767, round(ac.altitude / 25.0)))
        items[11] = fl.to_bytes(2, "big", signed=True)

    # FRN 15 — I021/155 Barometric Vertical Rate (signed 16-bit, LSB = 6.25 ft/min)
    if ac.vrate is not None:
        vr = max(-32_768, min(32_767, round(ac.vrate / 6.25)))
        items[15] = vr.to_bytes(2, "big", signed=True)

    # FRN 19 — I021/170 Track Angle (unsigned 16-bit, LSB = 360/2^16 °)
    direction = ac.track if ac.track is not None else ac.heading
    if direction is not None:
        ta = round(direction * 65536 / 360.0) & 0xFFFF
        items[19] = ta.to_bytes(2, "big")

    if not items:
        return b""

    # Build FSPEC bytes
    max_frn  = max(items)
    n_fspec  = (max_frn + 6) // 7
    fspec    = bytearray(n_fspec)
    for frn in items:
        bi, bp = _frn_bit(frn)
        fspec[bi] |= (1 << bp)
    for i in range(n_fspec - 1):
        fspec[i] |= 0x01        # set FX extension bit

    record    = bytes(fspec) + b"".join(items[f] for f in sorted(items))
    total_len = 3 + len(record)  # category(1) + length(2) + record
    return bytes([0x15]) + total_len.to_bytes(2, "big") + record


def cat021_lines(ac: Aircraft) -> list:
    """
    Return display rows for the CAT021 sidebar panel.
    Each entry: (item_id_str, value_str).
    """
    rows = []
    if ac.icao:
        label = f"{ac.callsign or '':<8}"
        rows.append(("I021/080", f"{ac.icao}  {label.strip()}"))
    if ac.lat is not None:
        rows.append(("I021/130", f"{ac.lat:+.4f}°"))
        rows.append(("        ", f"{ac.lon:+.4f}°"))
    if ac.altitude is not None:
        rows.append(("I021/140", f"{ac.altitude:,} ft"))
        rows.append(("I021/145", f"FL{ac.altitude // 100:03d}"))
    if ac.speed is not None:
        rows.append(("I021/160", f"{ac.speed} kt [{ac.speed_type}]"))
    direction = ac.track if ac.track is not None else ac.heading
    if direction is not None:
        rows.append(("I021/170", f"{direction:.1f}°"))
    if ac.vrate is not None:
        arrow = "▲" if ac.vrate > 0 else ("▼" if ac.vrate < 0 else "—")
        rows.append(("I021/155", f"{arrow}{abs(ac.vrate):,} fpm"))
    if ac.last_seen:
        age = (datetime.now(timezone.utc) - ac.last_seen).total_seconds()
        rows.append(("I021/030", f"{age:.0f}s ago"))
    return rows


# ═══════════════════════════════════════════════════════════════════════════════
# Section 2 — PPI coordinate transform
# ═══════════════════════════════════════════════════════════════════════════════

# Terminal character aspect ratio (row height / col width). Typical = 2.0.
# This is used so a circle looks round on screen rather than elliptical.
_CHAR_ASPECT = 2.0


class PPITransform:
    """Maps (lat, lon) → character-cell (col, row) in the PPI drawing area."""

    def __init__(self, lat0: float, lon0: float, range_nm: float,
                 cx: int, cy: int, col_r: int) -> None:
        self.lat0    = lat0
        self.lon0    = lon0
        self.range_nm = range_nm
        self.cx      = cx
        self.cy      = cy
        self.col_r   = col_r
        self.scale   = col_r / range_nm   # columns per nm

    def ll_to_cr(self, lat: float, lon: float) -> Optional[tuple]:
        """Return (col, row) for a lat/lon, or None if outside the display."""
        nm_e = (lon - self.lon0) * 60.0 * math.cos(math.radians(self.lat0))
        nm_n = (lat - self.lat0) * 60.0
        if math.hypot(nm_e, nm_n) > self.range_nm * 1.02:
            return None
        col = self.cx + round(nm_e * self.scale)
        row = self.cy - round(nm_n * self.scale / _CHAR_ASPECT)
        return col, row

    def bearing_to(self, lat: float, lon: float) -> float:
        """Bearing in degrees (clockwise from north) from centre to (lat, lon)."""
        nm_e = (lon - self.lon0) * 60.0 * math.cos(math.radians(self.lat0))
        nm_n = (lat - self.lat0) * 60.0
        return math.degrees(math.atan2(nm_e, nm_n)) % 360.0


# ═══════════════════════════════════════════════════════════════════════════════
# Section 3 — Curses helpers & colour pairs
# ═══════════════════════════════════════════════════════════════════════════════

_CP_HEADER  = 1   # cyan  — header bar / panel title
_CP_RING    = 2   # dim green — range rings & outer circle
_CP_SWEEP   = 3   # bright green — sweep line tip
_CP_TRAIL   = 4   # dim green — sweep trail
_CP_BLIP    = 5   # bright green+bold — fresh blip
_CP_FADE    = 6   # dim green — ageing blip
_CP_LABEL   = 7   # white — callsign / altitude labels
_CP_CATID   = 8   # cyan — CAT021 item IDs
_CP_CATVAL  = 9   # white — CAT021 values
_CP_STATUS  = 10  # yellow — status bar


def _init_colours() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(_CP_HEADER, curses.COLOR_CYAN,    -1)
    curses.init_pair(_CP_RING,   curses.COLOR_GREEN,   -1)
    curses.init_pair(_CP_SWEEP,  curses.COLOR_GREEN,   -1)
    curses.init_pair(_CP_TRAIL,  curses.COLOR_GREEN,   -1)
    curses.init_pair(_CP_BLIP,   curses.COLOR_GREEN,   -1)
    curses.init_pair(_CP_FADE,   curses.COLOR_GREEN,   -1)
    curses.init_pair(_CP_LABEL,  curses.COLOR_WHITE,   -1)
    curses.init_pair(_CP_CATID,  curses.COLOR_CYAN,    -1)
    curses.init_pair(_CP_CATVAL, curses.COLOR_WHITE,   -1)
    curses.init_pair(_CP_STATUS, curses.COLOR_YELLOW,  -1)


def _put(win, row: int, col: int, text: str, attr: int = 0) -> None:
    """Safe addstr — silently clips at window boundaries."""
    try:
        H, W = win.getmaxyx()
        if 0 <= row < H and 0 <= col < W:
            win.addstr(row, col, text[: W - col], attr)
    except curses.error:
        pass


# ═══════════════════════════════════════════════════════════════════════════════
# Section 4 — PPI drawing routines
# ═══════════════════════════════════════════════════════════════════════════════

def _ppi_point(cx: int, cy: int, col_r: int,
               angle_rad: float, r_frac: float) -> tuple:
    """Return (col, row) for a polar point inside the PPI circle."""
    col = cx + round(col_r * r_frac * math.sin(angle_rad))
    row = cy - round(col_r / _CHAR_ASPECT * r_frac * math.cos(angle_rad))
    return col, row


def draw_frame(win, cx: int, cy: int, col_r: int, range_nm: float) -> None:
    """Outer circle, range rings, cross-hairs, and cardinal labels."""
    ring_a  = curses.color_pair(_CP_RING)
    ring_b  = curses.color_pair(_CP_RING) | curses.A_BOLD
    # Outer circle  — every 2°
    for deg in range(0, 360, 2):
        c, r = _ppi_point(cx, cy, col_r, math.radians(deg), 1.0)
        _put(win, r, c, "·", ring_b)

    # Range rings
    step   = 50 if range_nm <= 400 else 100
    ring_r = step
    while ring_r < range_nm:
        frac = ring_r / range_nm
        for deg in range(0, 360, 5):
            c, r = _ppi_point(cx, cy, col_r, math.radians(deg), frac)
            _put(win, r, c, "·", ring_a)
        # Label near NE
        c, r = _ppi_point(cx, cy, col_r, math.radians(40), frac)
        _put(win, r - 1, c + 1, f"{ring_r}", ring_a)
        ring_r += step

    # Thin cross-hair lines (dashed)
    for frac in [i / 20 for i in range(1, 20)]:
        for ang in (0, 90, 180, 270):
            c, r = _ppi_point(cx, cy, col_r, math.radians(ang), frac)
            _put(win, r, c, "·", ring_a)

    # Cardinal labels
    N_r = cy - round(col_r / _CHAR_ASPECT) - 1
    S_r = cy + round(col_r / _CHAR_ASPECT) + 1
    _put(win, N_r,  cx,           "N",  ring_b | curses.A_BOLD)
    _put(win, S_r,  cx,           "S",  ring_b | curses.A_BOLD)
    _put(win, cy,   cx - col_r - 2, "W", ring_b | curses.A_BOLD)
    _put(win, cy,   cx + col_r + 1, "E", ring_b | curses.A_BOLD)
    _put(win, cy,   cx,           "+",  ring_b | curses.A_BOLD)


def draw_sweep(win, cx: int, cy: int, col_r: int, sweep_deg: float) -> None:
    """Rotating sweep line with a 20° fading trail behind it."""
    tip_a   = curses.color_pair(_CP_SWEEP) | curses.A_BOLD
    trail_a = curses.color_pair(_CP_TRAIL)
    trail_w = 20   # degrees of trailing glow

    for offset in range(-trail_w, 1):
        angle = math.radians((sweep_deg + offset) % 360)
        attr  = tip_a if offset == 0 else trail_a
        # Density: draw every other step for the trail, every step for the tip
        step  = 1 if offset == 0 else 2
        for i in range(step, col_r + 1, step):
            frac    = i / col_r
            col, row = _ppi_point(cx, cy, col_r, angle, frac)
            try:
                win.addch(row, col, ord("·"), attr)
            except curses.error:
                pass


def draw_tracks(win, fleet: dict, transform: PPITransform,
                blip_age: dict) -> None:
    """Aircraft blips (*) with callsign / flight-level labels."""
    blip_a  = curses.color_pair(_CP_BLIP)  | curses.A_BOLD
    fade1_a = curses.color_pair(_CP_FADE)  | curses.A_BOLD
    fade2_a = curses.color_pair(_CP_FADE)
    lbl_a   = curses.color_pair(_CP_LABEL)

    for icao, ac in list(fleet.items()):
        if ac.lat is None:
            continue
        cr = transform.ll_to_cr(ac.lat, ac.lon)
        if cr is None:
            continue
        col, row = cr
        age = blip_age.get(icao, 999.0)
        if age < 12.0:
            ch, attr = "◆", blip_a
        elif age < 24.0:
            ch, attr = "+", fade1_a
        else:
            ch, attr = "·", fade2_a

        _put(win, row, col, ch, attr)

        # Label one row above, one col right
        name    = (ac.callsign or icao).strip()
        alt_str = f"FL{ac.altitude // 100:03d}" if ac.altitude else "???"
        _put(win, row - 1, col + 1, f"{name}/{alt_str}", lbl_a)


def draw_cat021_panel(win, fleet: dict, x0: int, width: int, H: int) -> None:
    """ASTERIX CAT021 data panel drawn at column x0."""
    hdr_a = curses.color_pair(_CP_HEADER) | curses.A_BOLD
    id_a  = curses.color_pair(_CP_CATID)
    val_a = curses.color_pair(_CP_CATVAL)
    sep_a = curses.color_pair(_CP_RING)

    _put(win, 1, x0, "─" * width, sep_a)
    _put(win, 2, x0, " ASTERIX CAT021  Ed 2.1", hdr_a)
    _put(win, 3, x0, "─" * width, sep_a)

    row = 4
    for ac in sorted(fleet.values(), key=lambda a: a.icao):
        if row >= H - 2:
            _put(win, row, x0, f" … +{len(fleet)} more", id_a)
            break
        # Aircraft header row
        name = f"{ac.icao}  {(ac.callsign or '').strip()}"
        _put(win, row, x0, f" {name:<{width-1}}", hdr_a | curses.A_BOLD)
        row += 1
        for item_id, value in cat021_lines(ac):
            if row >= H - 2:
                break
            _put(win, row, x0,      f"  {item_id}", id_a)
            _put(win, row, x0 + 12, value[:width - 13], val_a)
            row += 1
        _put(win, row, x0, "─" * width, sep_a)
        row += 1


# ═══════════════════════════════════════════════════════════════════════════════
# Section 5 — UDP multicast receiver thread
# ═══════════════════════════════════════════════════════════════════════════════

def _receiver(group: str, port: int, iface: str,
              fleet: dict, lock: threading.Lock,
              ast_fh) -> None:
    """Receive raw ADS-B UDP datagrams and update the shared fleet dict."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    mreq = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton(iface))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(0.5)
    buf = ""
    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                buf += data.decode("ascii", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    with lock:
                        res = decode_message(line, fleet)
                        if res.get("valid") and ast_fh:
                            icao = res.get("icao")
                            if icao and icao in fleet:
                                block = encode_cat021(fleet[icao])
                                if block:
                                    ast_fh.write(block)
                                    ast_fh.flush()
            except socket.timeout:
                pass
    finally:
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
        except OSError:
            pass
        sock.close()


# ═══════════════════════════════════════════════════════════════════════════════
# Section 6 — Main display loop
# ═══════════════════════════════════════════════════════════════════════════════

_SWEEP_DEG_PER_SEC = 36.0   # 1 rotation per 10 seconds
_PANEL_COLS        = 34     # width of CAT021 side panel


def _auto_centre(fleet: dict) -> tuple:
    lats = [a.lat for a in fleet.values() if a.lat is not None]
    lons = [a.lon for a in fleet.values() if a.lon is not None]
    if not lats:
        return 0.0, 0.0
    return sum(lats) / len(lats), sum(lons) / len(lons)


def _run(stdscr, group: str, port: int, iface: str,
         fixed_centre: Optional[tuple], range_nm: float, ast_fh) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    _init_colours()

    fleet      = {}
    lock       = threading.Lock()
    blip_age   = {}     # icao → seconds since last sweep illumination
    sweep_deg  = 0.0
    last_tick  = time.monotonic()
    n_received = 0

    rx = threading.Thread(
        target=_receiver,
        args=(group, port, iface, fleet, lock, ast_fh),
        daemon=True,
    )
    rx.start()

    hdr_a    = curses.color_pair(_CP_HEADER) | curses.A_BOLD
    status_a = curses.color_pair(_CP_STATUS)
    ring_a   = curses.color_pair(_CP_RING)

    while True:
        H, W = stdscr.getmaxyx()
        if H < 20 or W < 60:
            stdscr.erase()
            _put(stdscr, 0, 0, f"Terminal too small ({W}×{H}). Need ≥ 60×20.", hdr_a)
            stdscr.refresh()
            time.sleep(0.5)
            key = stdscr.getch()
            if key in (ord("q"), ord("Q")):
                break
            continue

        # ── Key input ──────────────────────────────────────────────────────
        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            break
        elif key in (ord("+"), ord("=")):
            range_nm = max(25.0, range_nm * 0.75)
        elif key in (ord("-"), ord("_")):
            range_nm = min(3000.0, range_nm / 0.75)

        # ── Time step ──────────────────────────────────────────────────────
        now       = time.monotonic()
        dt        = now - last_tick
        last_tick = now
        sweep_deg = (sweep_deg + _SWEEP_DEG_PER_SEC * dt) % 360.0

        # ── Age blips & illuminate those swept over ────────────────────────
        with lock:
            for icao in list(blip_age):
                blip_age[icao] += dt

            clat, clon = fixed_centre if fixed_centre else _auto_centre(fleet)
            xfm = PPITransform(clat, clon, range_nm,
                               cx=0, cy=0, col_r=1)   # temp, for bearing calc
            for icao, ac in fleet.items():
                if ac.lat is None:
                    continue
                brng = xfm.bearing_to(ac.lat, ac.lon)
                diff = (sweep_deg - brng) % 360.0
                if diff <= _SWEEP_DEG_PER_SEC * dt + 2.0:
                    blip_age[icao] = 0.0

            n_tracks = sum(1 for a in fleet.values() if a.lat is not None)

        # ── Layout geometry ────────────────────────────────────────────────
        ppi_w  = max(20, W - _PANEL_COLS)
        cx     = ppi_w // 2
        # Two header rows (0, 1) + PPI area + two footer rows
        ppi_h  = H - 4          # rows available for PPI
        cy     = 2 + ppi_h // 2
        # col_r must satisfy: 2*col_r < ppi_w  AND  col_r < ppi_h
        col_r  = min(ppi_w // 2 - 2, ppi_h - 1)

        transform = PPITransform(clat, clon, range_nm, cx, cy, col_r)

        # ── Draw ───────────────────────────────────────────────────────────
        stdscr.erase()

        # Header bar
        ts  = datetime.now(timezone.utc).strftime("%H:%M:%S UTC")
        hdr = (f"  ADS-B PPI  ◈  {group}:{port}  "
               f"◈  {n_tracks} track{'s' if n_tracks != 1 else ''}  "
               f"◈  range {range_nm:.0f} nm  ◈  {ts}")
        _put(stdscr, 0, 0, "─" * W,     ring_a)
        _put(stdscr, 1, 0, hdr[:W],     hdr_a)

        # PPI elements
        draw_frame(stdscr, cx, cy, col_r, range_nm)
        draw_sweep(stdscr, cx, cy, col_r, sweep_deg)
        with lock:
            draw_tracks(stdscr, fleet, transform, blip_age)
            draw_cat021_panel(stdscr, fleet, ppi_w, _PANEL_COLS, H)

        # Vertical divider between PPI and panel
        for r in range(2, H - 2):
            _put(stdscr, r, ppi_w - 1, "│", ring_a)

        # Footer bar
        ast_note = f"  ASTERIX → {ast_fh.name}" if ast_fh else ""
        status   = f"  [+/-] zoom  [q] quit{ast_note}"
        _put(stdscr, H - 2, 0, "─" * W,     ring_a)
        _put(stdscr, H - 1, 0, status[:W],   status_a)

        stdscr.refresh()
        time.sleep(0.08)   # ≈ 12 fps


# ═══════════════════════════════════════════════════════════════════════════════
# Section 7 — Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="ADS-B PPI display with ASTERIX CAT021",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--group",   default="239.255.0.1",
                        help="Multicast group (default: 239.255.0.1)")
    parser.add_argument("--port",    type=int, default=30003,
                        help="UDP port (default: 30003)")
    parser.add_argument("--iface",   default="0.0.0.0",
                        help="Local interface (default: 0.0.0.0)")
    parser.add_argument("--centre",  metavar="LAT,LON",
                        help="Fixed radar centre, e.g. 52.0,4.0")
    parser.add_argument("--range",   type=float, default=300.0,
                        help="Initial display range in nm (default: 300)")
    parser.add_argument("--asterix", metavar="FILE",
                        help="Write binary ASTERIX CAT021 records to FILE")
    args = parser.parse_args()

    fixed_centre = None
    if args.centre:
        lat_s, lon_s = args.centre.split(",")
        fixed_centre = float(lat_s), float(lon_s)

    ast_fh = open(args.asterix, "wb") if args.asterix else None
    try:
        curses.wrapper(
            _run,
            args.group, args.port, args.iface,
            fixed_centre, args.range, ast_fh,
        )
    finally:
        if ast_fh:
            ast_fh.close()


if __name__ == "__main__":
    main()
