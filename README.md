# ADS-B Toolkit

A self-contained Python toolkit for generating, transmitting, receiving, and
displaying ADS-B (Automatic Dependent Surveillance–Broadcast) messages over
UDP multicast.

---

## Overview

```
aircraft_emulator.py  ──UDP multicast──►  adsb_decoder.py
                                               │
interactive_emulator.py ──UDP multicast──►    │  (shared fleet model)
                                               │
                                          ppi_display.py
```

| File | Role |
|---|---|
| `adsb_decoder.py` | Decode raw 1090 MHz Mode S Extended Squitter frames |
| `aircraft_emulator.py` | Scripted fleet emulator — transmits ADS-B over UDP multicast |
| `interactive_emulator.py` | GUI emulator — click a PPI radar to place and move aircraft |
| `ppi_display.py` | Curses radar display with ASTERIX CAT021 side panel |

---

## Requirements

- Python 3.10+ with tkinter (system Python recommended on macOS)
- [pyModeS](https://github.com/junzis/pyModeS) ≥ 3.3 — table-driven CRC-24

```bash
uv venv --python /Library/Frameworks/Python.framework/Versions/3.13/bin/python3 .venv
uv pip install pyModeS
```

---

## Quick start

### 1 — Scripted emulator + live coordinate table

Terminal 1 (emulator):
```bash
source .venv/bin/activate
python aircraft_emulator.py          # 3 aircraft, 239.255.0.1:30003
python aircraft_emulator.py --count 4 --rate 4
```

Terminal 2 (decoder — refreshes every second):
```bash
source .venv/bin/activate
python adsb_decoder.py --multicast
```

Sample output:
```
ICAO    Callsign      Latitude    Longitude   Alt ft  Spd kt  Track°    Seen
────────────────────────────────────────────────────────────────────────────
4840D6  KLM1023     +52.00001°    +3.99997°   38,000     479    45.0      1s
7C4516  QFA007      -33.87001°  +151.20998°   12,000     249   189.9      0s
A05F21  UAL456      +40.70998°   -73.99997°   35,000     520   270.0      1s

  14:32:01 UTC   239.255.0.1:30003   3 tracked
```

Aircraft stop appearing after 30 seconds without a message.

### 2 — Interactive emulator

```bash
source .venv/bin/activate
python interactive_emulator.py
python interactive_emulator.py --centre 51.5,-0.5 --range 150
```

- **Left-click** inside the radar circle → place a new aircraft
- **Drag** an existing blip → reposition it
- **Right-click** a blip → delete it
- Use the right panel to edit altitude / speed / heading of the selected track

The interactive emulator transmits to the same multicast group and port, so
the decoder and PPI display see its aircraft alongside the scripted ones.

### 3 — Curses PPI display with ASTERIX CAT021

```bash
source .venv/bin/activate
python ppi_display.py
python ppi_display.py --centre 52.0,4.0 --range 200
python ppi_display.py --asterix out.ast    # also write binary ASTERIX records
```

Keys: `+`/`=` zoom in · `-` zoom out · `q` quit

---

## Decoder modes

```bash
python adsb_decoder.py                      # decode four built-in demo messages
python adsb_decoder.py --msg 8D4840D6...    # decode one hex message
python adsb_decoder.py --file msgs.txt      # decode a newline-separated hex file
python adsb_decoder.py --live               # TCP stream from dump1090 :30002
python adsb_decoder.py --multicast          # UDP multicast 239.255.0.1:30003
python adsb_decoder.py --multicast --group 239.255.0.1 --iface 127.0.0.1
```

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
| 19 | Airborne Velocity — ground speed or airspeed + track + V/S |
| 20–22 | Airborne Position — GNSS altitude + CPR lat/lon |

---

## ASTERIX CAT021

`ppi_display.py` encodes each track as an ASTERIX CAT021 Edition 2.1 binary
record and optionally writes it to a file (`--asterix`).

Items encoded (when data is available):

| FRN | Item | Description |
|---|---|---|
| 1 | I021/010 | Data Source Identifier (SAC/SIC) |
| 3 | I021/030 | Time of Day (1/128 s since midnight UTC) |
| 4 | I021/130 | Position in WGS-84 (LSB = 180/2²³ °) |
| 5 | I021/080 | Target Address (24-bit ICAO) |
| 6 | I021/140 | Geometric Height (LSB = 6.25 ft) |
| 11 | I021/145 | Flight Level (LSB = 25 ft) |
| 15 | I021/155 | Barometric Vertical Rate (LSB = 6.25 ft/min) |
| 19 | I021/170 | Track Angle (LSB = 360/2¹⁶ °) |

---

## Architecture notes

**CPR position encoding/decoding** — Compact Position Reporting uses two
coprime zone grids (60 even, 59 odd) so an even+odd pair uniquely resolves the
aircraft's global position via the Chinese Remainder Theorem.  Single frames
are ambiguous; the decoder waits for a matching pair within 10 seconds.

**CRC-24** — pyModeS provides a table-driven implementation.
`crc_remainder(int(msg, 16), 112) == 0` validates a complete 112-bit frame.
For signing emitted messages: append `"000000"` (zero parity) and compute
`crc_remainder(int(payload + "000000", 16), 112)`; the result is the
correct 24-bit parity to append.

**Threading in the interactive emulator** — the TX thread writes only to
`self._tx_status` (a plain Python string); the main tkinter loop reads it once
per frame and updates the label.  This avoids calling `widget.after()` or any
tkinter API from a non-main thread, which can deadlock the event queue.
