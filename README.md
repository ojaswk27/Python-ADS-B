# ADS-B Toolkit

A self-contained Python toolkit for generating, transmitting, receiving, decoding,
and displaying ADS-B (Automatic Dependent Surveillance–Broadcast) messages over
UDP multicast. Two GUI tools sit on top of an emulator and a hand-rolled decoder,
and the path emulator additionally emits ASTERIX CAT021 target reports plus a
small custom binary message that announces the radar's own location.

All ADS-B traffic flows on a single multicast group/port from `network.cfg`;
ASTERIX traffic is unicast to a separately configurable destination.

---

## Tools

| File | Role |
|---|---|
| `path_emulator.py` | GUI path editor — draw waypoint routes (or freehand strokes); aircraft fly the paths and the program transmits ADS-B + ASTERIX CAT021 + radar-position UDP |
| `radar_display.py` | GUI radar receiver — live PPI with fading trails, random per-target colours, track panel |
| `radar_ui.py` | Shared UI module — palette, fonts, geometry, scale-aware drawing primitives |
| `adsb_decoder.py` | Pure-stdlib Mode S decoder — single hex, file, dump1090 TCP, or UDP multicast |
| `aircraft_emulator.py` | Headless scripted fleet emulator (no GUI) |
| `cat21.py` | Minimal ASTERIX CAT021 (Target Reports) encoder used by `path_emulator.py` |
| `net_config.py` | Shared config loader for `network.cfg` |
| `test_velocity.py` | unittest suite covering velocity-message encode/decode round-trips |
| `decoder_pseudocode.txt` | Plain-English walkthrough of the decoder's core for reference |

---

## Requirements

- Python 3.10+ with Tk (system Python on macOS works out of the box)
- [pyModeS](https://github.com/junzis/pyModeS) ≥ 3.3 (used for the CRC-24 helper and end-to-end test decoding)

```bash
uv venv
uv pip install -r requirements.txt
source .venv/bin/activate
```

---

## Running the tools

All four programs print full `--help` text. Anything not given on the command line
falls back to `network.cfg`, then to a hard-coded default.

### `path_emulator.py` — GUI sensor / emulator

```bash
python path_emulator.py
python path_emulator.py --centre 51.5,-0.5 --range 150
python path_emulator.py --declination 1.5 \
                       --asterix-host 192.168.1.20 --asterix-port 8600
```

| Flag | Default | Purpose |
|---|---|---|
| `--group GROUP` | `network.cfg` (`239.255.0.1`) | ADS-B multicast group to transmit on |
| `--port PORT` | `network.cfg` (`30003`) | ADS-B multicast port |
| `--iface IFACE` | `network.cfg` (`127.0.0.1`) | Local interface for multicast |
| `--centre LAT,LON` | `51.477,-0.461` | Centre of the radar PPI (sensor location) |
| `--range RANGE` | `200.0` | Disc radius in nautical miles |
| `--declination DEG` | `0.0` | Magnetic declination °E; converts true track → magnetic for the on-screen / CAT21 readout |
| `--asterix-host IP` | `network.cfg` (`127.0.0.1`) | Unicast IP for CAT021 + the 12-byte radar-position message |
| `--asterix-port PORT` | `network.cfg` (`8600`) | Unicast port for the same |

**Controls**

| Action | Result |
|---|---|
| Left-click empty space | drop one waypoint (auto-creates an aircraft if none selected) |
| Click-drag empty space | freehand path — points sampled at uniform spacing along the drag |
| Drag a waypoint dot | reposition it (only works once that aircraft is selected) |
| Right-click a waypoint | delete it |
| Right-click empty canvas | toggle the hover crosshair on/off |
| Aircraft list / address / callsign fields | select and rename a track |
| alt / speed sliders | live update of the selected aircraft |
| loop checkbox | close the path into a loop (off = fly start→end and hold) |
| Hover | crosshair with exact lat/lon under the pointer |
| F11 / Esc | toggle / leave fullscreen (radar autoscales) |
| Reset positions (red) | rewind every aircraft to the start of its path |

**Outputs from `path_emulator.py`**

| Stream | Destination | Cadence |
|---|---|---|
| ADS-B raw hex (`*HEX;\n`) | multicast `--group:--port` | continuous, ~per-aircraft round-robin |
| ASTERIX CAT021 data blocks | unicast `--asterix-host:--asterix-port` | one block per second containing one record per active target |
| 12-byte radar-position message | unicast `--asterix-host:--asterix-port` | 8 packets at 500 ms intervals during startup (4 s burst) |

### `radar_display.py` — GUI radar receiver

```bash
python radar_display.py
python radar_display.py --centre 51.5,-0.5 --range 150
```

| Flag | Default | Purpose |
|---|---|---|
| `--group GROUP` | `network.cfg` (`239.255.0.1`) | ADS-B multicast group to listen on |
| `--port PORT` | `network.cfg` (`30003`) | ADS-B multicast port |
| `--iface IFACE` | `network.cfg` (`127.0.0.1`) | Local interface for multicast |
| `--centre LAT,LON` | `51.477,-0.461` | Centre of the PPI |
| `--range RANGE` | `200.0` | Disc radius in nautical miles |

**Controls**

| Action | Result |
|---|---|
| lat / lon / range entries + Apply | recentre/rescale the PPI |
| Clear screen (red) | drop all current tracks and trails (RX keeps running) |
| F11 / Esc | toggle / leave fullscreen (radar autoscales) |

Each target is assigned a random vivid colour on first sight (golden-ratio hue
spread to avoid lookalikes); the colour fades with message age (0–3 s full,
3–10 s mid, 10–30 s dim, then culled).

### `aircraft_emulator.py` — headless scripted fleet

```bash
python aircraft_emulator.py                       # 3 scripted aircraft
python aircraft_emulator.py --count 4 --rate 4    # busier feed
```

| Flag | Default | Purpose |
|---|---|---|
| `--group GROUP` | `network.cfg` | Multicast group |
| `--port PORT` | `network.cfg` | Multicast port |
| `--iface IFACE` | `network.cfg` | Local interface |
| `--count COUNT` | `3` | Number of pre-defined aircraft (1–4) |
| `--rate RATE` | `2` | Messages per second per aircraft |

### `adsb_decoder.py` — pure-stdlib decoder

```bash
python adsb_decoder.py                          # decode built-in demo messages
python adsb_decoder.py --msg 8D4840D6202CC371C32CE0576098
python adsb_decoder.py --file msgs.txt
python adsb_decoder.py --live                   # dump1090 TCP :30002
python adsb_decoder.py --multicast              # listen to network.cfg multicast
```

| Flag | Default | Purpose |
|---|---|---|
| `--msg HEX` | — | Decode one 28-char hex frame and exit |
| `--file PATH` | — | Decode a file of newline-separated hex frames |
| `--live` | — | Live TCP stream from dump1090 |
| `--multicast` | — | Live UDP multicast stream (this toolkit's default mode) |
| `--host HOST` | `127.0.0.1` | TCP host for `--live` |
| `--port PORT` | `30002` (live) / `network.cfg` (multicast) | Port override |
| `--group GROUP` | `network.cfg` | Multicast group for `--multicast` |
| `--iface IFACE` | `network.cfg` | Local interface for `--multicast` |

`--msg`, `--file`, `--live`, `--multicast` are mutually exclusive; with none of
them the decoder runs through its built-in demo frames.

### Tests

```bash
python -m unittest test_velocity -v
```

---

## Network configuration

Edit `network.cfg` to change defaults. Every CLI flag still overrides whatever is
in this file.

```ini
# ADS-B multicast network settings
group = 239.255.0.1
port  = 30003
iface = 127.0.0.1     # loopback — change to a LAN interface for real hardware

# ASTERIX CAT021 + radar-position output (path_emulator, unicast UDP)
asterix_host = 127.0.0.1
asterix_port = 8600
```

---

## Output formats

### ADS-B raw hex (multicast)

dump1090-compatible framing:

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

Type codes the toolkit emits/decodes:

| TC | Message type |
|---|---|
| 1–4 | Aircraft Identification (callsign + wake category) |
| 9–18 | Airborne Position — barometric altitude + CPR lat/lon |
| 19 | Airborne Velocity — ground speed, track angle, vertical rate |
| 20–22 | Airborne Position — GNSS altitude + CPR lat/lon |

### ASTERIX CAT021 (unicast)

`path_emulator.py` emits one ASTERIX data block per second to
`--asterix-host:--asterix-port`. Each block contains one record per active
target. The encoder lives in `cat21.py` and implements just the items the
toolkit needs (FSPEC is built automatically from whichever items are present):

| FRN | Data item | Contents |
|---|---|---|
| 1 | I021/010 | Data Source Identification (SAC/SIC) |
| 2 | I021/040 | Target Report Descriptor |
| 3 | I021/161 | Track Number |
| 6 | I021/130 | Position in WGS-84 (24-bit lat + 24-bit lon) |
| 11 | I021/080 | Target Address (24-bit ICAO) |
| 16 | I021/140 | Geometric Height |
| 21 | I021/145 | Flight Level |
| 22 | I021/152 | Magnetic Heading |
| 26 | I021/160 | Airborne Ground Vector (speed + true track) |
| 29 | I021/170 | Target Identification (callsign) |

Reference: EUROCONTROL ASTERIX Part 12, Category 021 Ed. 2.x. All multi-byte
fields are big-endian (network byte order), matching the ASTERIX spec.

### Custom 12-byte radar-position message (unicast)

At startup, `path_emulator.py` sends **8 copies** of a small fixed-layout binary
message to `--asterix-host:--asterix-port`, **one every 500 ms** for a total of
4 seconds. This announces the radar's own location to receivers on the same
unicast channel.

| Offset | Field | Type | Value |
|---|---|---|---|
| 0–1 | `size` | `uint16` | always `12` (length of the packet itself) |
| 2–3 | `messageId` | `uint16` | always `0x1306` |
| 4–7 | `latitude` | `float32` | radar centre, decimal degrees, WGS-84 |
| 8–11 | `longitude` | `float32` | radar centre, decimal degrees, WGS-84 |

**Byte order: little-endian.** Both x86 and Raspberry Pi / ARM Linux run
little-endian, so a receiver written as a plain C struct will read the fields
directly with no swapping on either platform. With Python's `struct` module the
format string is `<HHff`.

C-style definition for reference:

```c
#pragma pack(push, 1)
typedef struct {
    uint16_t size;        // = 12
    uint16_t messageId;   // = 0x1306
    float    latitude;
    float    longitude;
} RadarPositionMsg;       // 12 bytes
#pragma pack(pop)
```

The burst is fire-and-forget — there's no ack, no retransmission beyond the
initial 8 packets. Receivers that come up later won't see it; restart the
emulator if you need to re-announce.

---

## UI scaling

`radar_ui.SCALE` (default `2`) sets the baseline pixel density for both GUI
tools. On top of that, both windows are resizable and the radar disc, blips,
labels, and fonts all scale dynamically with the live canvas size — F11 enters
fullscreen, Esc leaves it. The side panel keeps a fixed width for readability.

---

## Architecture notes

**CPR position encoding/decoding** — Compact Position Reporting uses two
coprime zone grids (60 even, 59 odd) so an even+odd pair uniquely resolves the
aircraft's global position via the Chinese Remainder Theorem. Single frames are
ambiguous; the decoder pairs them within a 10-second window keyed by ICAO. The
newer of the pair drives the reported lat/lon.

**CRC-24** — pyModeS provides a table-driven implementation.
`crc_remainder(int(msg, 16), 112) == 0` validates a complete 112-bit frame.
For signing emitted messages: append `"000000"` and compute
`crc_remainder(int(payload + "000000", 16), 112)`; the result is the 24-bit
parity field.

**Velocity message subtypes** — the emulator auto-switches between subtype 1
(subsonic, 1 kt/LSB, ≤1022 kt) and subtype 2 (supersonic, 4 kt/LSB,
≤4088 kt). Component magnitudes are rounded (not truncated) so quantisation
error is centred around zero.

**Threading model** — TX and RX threads write only to plain Python attributes
guarded by a lock; all tkinter calls happen on the main thread. The render
loop is layered: a cached background (rings + lat/lon grid) and a cached route
layer are rebuilt only on view/edit changes; only the per-frame blip/label
layer is redrawn each tick. This keeps a fullscreen Retina canvas from
recompositing the whole scene every frame.

**Per-target colours** — both tools draw each track in a vivid hue chosen from
a low-discrepancy (golden-ratio) sequence, so two simultaneously visible
targets never land on near-identical colours.
