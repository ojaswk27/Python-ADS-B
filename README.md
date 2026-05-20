# ADS-B Toolkit

A self-contained Python toolkit for generating, transmitting, receiving, and
displaying ADS-B (Automatic Dependent Surveillance–Broadcast) messages over
UDP multicast. All tools communicate over a single configurable multicast
address defined in `network.cfg`.

---

## Tools

| File | Role |
|---|---|
| `path_emulator.py` | GUI path editor — draw waypoint routes, aircraft fly them and transmit ADS-B |
| `radar_display.py` | GUI radar receiver — live PPI with rotating sweep, fading trails, track panel |
| `radar_ui.py` | Shared UI module — palette, fonts, geometry helpers, canvas drawing primitives |
| `adsb_decoder.py` | Decode raw Mode S frames; live multicast or file input |
| `aircraft_emulator.py` | Headless scripted fleet emulator |
| `interactive_emulator.py` | GUI emulator — click a PPI to place and move aircraft manually |
| `ppi_display.py` | Curses radar display with ASTERIX CAT021 side panel |
| `net_config.py` | Shared config loader for `network.cfg` |

---

## Requirements

- Python 3.10+ with tkinter (system Python recommended on macOS)
- [pyModeS](https://github.com/junzis/pyModeS) ≥ 3.3

```bash
uv venv --python /Library/Frameworks/Python.framework/Versions/3.13/bin/python3 .venv
uv pip install pyModeS
```

---

## Quick start

### Path emulator + radar display (recommended)

Terminal 1 — draw flight paths and transmit:
```bash
source .venv/bin/activate
python path_emulator.py
```

Terminal 2 — receive and display:
```bash
source .venv/bin/activate
python radar_display.py
```

**Path emulator controls:**
- **Left-click** inside the radar circle → add a waypoint (auto-creates an aircraft)
- **Drag** a waypoint dot → reposition it
- **Right-click** a waypoint → delete it
- Select an aircraft in the list, edit altitude / speed, press **Apply**
- Aircraft must have ≥ 2 waypoints to start flying

### Headless emulator + live coordinate table

```bash
source .venv/bin/activate
python aircraft_emulator.py          # 3 scripted aircraft
python aircraft_emulator.py --count 4 --rate 4
```

```bash
source .venv/bin/activate
python adsb_decoder.py --multicast
```

Sample decoder output:
```
ICAO    Callsign      Latitude    Longitude   Alt ft  Spd kt  Track°    Seen
────────────────────────────────────────────────────────────────────────────
4840D6  KLM1023     +52.00001°    +3.99997°   38,000     479    45.0      1s
7C4516  QFA007      -33.87001°  +151.20998°   12,000     249   189.9      0s
A05F21  UAL456      +40.70998°   -73.99997°   35,000     520   270.0      1s
```

---

## Network configuration

Edit `network.cfg` to change the multicast group, port, or interface. All tools
read this file at startup via `net_config.py`.

```ini
group = 239.255.0.1
port  = 30003
iface = 127.0.0.1   # loopback — change to a LAN interface for real hardware
```

All tools also accept `--group`, `--port`, `--iface` command-line overrides.

---

## UI scaling

All display sizes, fonts, and pixel offsets are derived from a single constant
in `radar_ui.py`:

```python
SCALE = 2   # 1 = compact  ·  2 = double (default)
```

Changing `SCALE` and restarting resizes both `path_emulator.py` and
`radar_display.py` uniformly.

---

## Message format

All tools exchange messages in dump1090 raw format:

```
*<28-hex-chars>;\n
```

Example: `*8D4840D6232CC371C32CC0CDDA38;\n`

The 28-character hex string is a 112-bit Mode S Extended Squitter (DF=17):

```
Bits   1– 5   DF   Downlink Format (17 = ADS-B ES)
Bits   6– 8   CA   Capability
Bits   9–32   ICAO 24-bit aircraft address
Bits  33–88   ME   Extended Squitter payload (56 bits)
               Bits 33–37 : Type Code (TC)
Bits  89–112  CRC-24
```

### Type Codes decoded

| TC | Message type |
|---|---|
| 1–4 | Aircraft Identification (callsign + wake category) |
| 9–18 | Airborne Position — barometric altitude + CPR lat/lon |
| 19 | Airborne Velocity — ground speed, track angle, vertical rate |
| 20–22 | Airborne Position — GNSS altitude + CPR lat/lon |

---

## Architecture notes

**CPR position encoding/decoding** — Compact Position Reporting uses two
coprime zone grids (60 even, 59 odd) so an even+odd pair uniquely resolves the
aircraft's global position. Single frames are ambiguous; the decoder pairs them
within a 10-second window.

**CRC-24** — pyModeS provides a table-driven implementation.
`crc_remainder(int(msg, 16), 112) == 0` validates a complete 112-bit frame.
For signing emitted messages: append `"000000"` and compute
`crc_remainder(int(payload + "000000", 16), 112)`; the result is the
24-bit parity field.

**Threading model** — background TX/RX threads write only to plain Python
string/dict attributes; the main tkinter loop reads them once per frame.
This avoids calling any tkinter API from a non-main thread, which can
deadlock the event queue.

**ASTERIX CAT021** — `ppi_display.py` encodes each track as an ASTERIX
CAT021 Edition 2.1 binary record and optionally writes it to a file
(`--asterix`). Items encoded: I021/010 (source), I021/030 (time),
I021/130 (position), I021/080 (ICAO), I021/140 (geometric height),
I021/145 (flight level), I021/155 (V/S), I021/170 (track angle).
