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
| `simulator.py` | **GUI, single process** — airspace, IFF/SSR interrogator and 1090 MHz receiver in one window. Aircraft emit encoded frames; a channel loses or corrupts them; a receiver builds the whole display by decoding what survives |
| `channel.py` | Propagation: max range, radio horizon, frame loss, round-trip delay, Mode A/C garble |
| `receiver.py` | Owns the track store. Measured range and bearing, CPR resolution. No ground-truth access |
| `pseudo1090.py` + `pseudo1090.cfg` | **Custom 1090 MHz message format** — a config-defined format that replaces ADS-B on the same physical layer, so no new hardware is needed. `--check` validates a config, `--demo` round-trips one |
| `iff_protocol.py` | IFF reply codec, Mode C pressure altitude, Mode 1 codes, shared geometry |
| `path_emulator.py` | GUI path editor — draw waypoint routes (or freehand strokes); aircraft fly the paths and the program transmits ADS-B + ASTERIX CAT021 + radar-position UDP |
| `radar_display.py` | GUI radar receiver — live PPI with fading trails, random per-target colours, track panel |
| `radar_ui.py` | Shared UI module — palette, fonts, geometry, scale-aware drawing primitives |
| `adsb_decoder.py` | Pure-stdlib Mode S decoder — single hex, file, dump1090 TCP, or UDP multicast |
| `aircraft_emulator.py` | Headless scripted fleet emulator (no GUI), plus the ADS-B frame builders and `decode_frame` |
| `cat21.py` | Minimal ASTERIX CAT021 (Target Reports) encoder used by `path_emulator.py` |
| `net_config.py` | Shared config loader for `network.cfg` |
| `acceptance/` | Suites driving the real application; `python acceptance/run_all.py` |
| `test_codecs.py`, `test_pseudo1090.py`, `test_velocity.py` | Codec round-trip tests, no display needed |
| `decoder_pseudocode.txt` | Plain-English walkthrough of the decoder's core for reference |

---

## Custom 1090 MHz format

`simulator.py` can transmit a **custom message format in place of ADS-B**, reusing
the ADS-B physical layer — same 1090 MHz transceivers, same 112-bit Mode S
frame, same CRC-24 — so no new hardware is required. Only the payload content
changes: all 88 bits are defined by `pseudo1090.cfg`.

```
frame = [ 88 payload bits, defined by the cfg ][ 24-bit CRC ]
```

Edit `pseudo1090.cfg` to define message types, their bit-field layouts, and
their transmit periods, then check it without launching the GUI:

```bash
python pseudo1090.py --check      # layout, bit map, resolutions, all errors at once
python pseudo1090.py --demo       # encode and decode a sample of each message
```

Switch formats with the `transmit` control in the **1090 FORMAT** panel:

| Mode | Transmits |
|---|---|
| `standard` | DO-260B ADS-B only — the original behaviour, and the shipped default |
| `custom` | the config's format only, replacing ADS-B entirely |
| `both` | both, for comparing them side by side on one screen |

The **RADAR SITE** panel moves and resizes the radar at runtime — centre
latitude/longitude, display range, and antenna height (which feeds the radio
horizon). **Centre on selected** puts the site under the selected aircraft.
Relocating clears existing tracks, because a stored plot is a range and bearing
measured *from the old position*; reprojecting it about the new one would place
the target somewhere it never was.

The **1090 AIRTIME** pane logs every transmission with its decode result, so a
frame lost in the channel or rejected by the decoder is visible rather than
silently absent. `--format-cfg PATH` selects a different config; **Reload
format cfg** re-reads it without restarting.

Two consequences of owning all 88 bits, both enforced by `--check` rather than
left to surprise you: with more than one message type the config must declare a
`type_field`, since nothing else on the wire distinguishes them; and a decoded
message cannot be tied to an aircraft unless the config says which bits carry
the address.

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

`path_emulator.py` emits three UDP streams: a public ADS-B raw hex feed
(dump1090-compatible framing), an ASTERIX CAT021 target-report block, and a
short startup burst announcing the radar's own position. Byte-level layouts
are not included in this README; refer to the source in `cat21.py`,
`iff_protocol.py`, and `iff_interrogation.py` for the encoded fields.

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
