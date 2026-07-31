# IFF Radar + ADS-B Simulator

A self-contained Python simulator for IFF/SSR interrogation and 1090 MHz
downlink. One process holds the whole chain: aircraft emit encoded frames, a
propagation channel loses or corrupts them, and a receiver builds the entire
display by decoding only what survives. Nothing on screen reads ground truth.

```bash
python simulator.py
```

---

## Files

Everything at the top level is part of the running simulator. Superseded
programs live in `legacy/` and are not imported by anything here.

| File | Role |
|---|---|
| `simulator.py` | **The program.** Airspace, IFF/SSR interrogator, and 1090 MHz receiver in one window |
| `channel.py` | Propagation: max range, radio horizon, frame loss, round-trip delay, monopulse azimuth, Mode A/C garble |
| `receiver.py` | Owns the track store. Measured range and bearing, CPR resolution. No ground-truth access |
| `iff_protocol.py` | IFF reply codec, Mode C pressure altitude, Mode 1 codes, shared geometry |
| `aircraft_emulator.py` | ADS-B frame builders and `decode_frame`; also runs standalone as a headless fleet emitter |
| `adsb_decoder.py` | Pure-stdlib Mode S decoder — single hex, file, dump1090 TCP, or UDP multicast |
| `pseudo1090.py` + `pseudo1090.cfg` | **Custom 1090 MHz message format** — a config-defined format that replaces ADS-B on the same physical layer, so no new hardware is needed. `--check` validates a config, `--demo` round-trips one |
| `radar_ui.py` | Shared UI module — palette, fonts, geometry, scale-aware drawing primitives |
| `net_config.py` + `network.cfg` | Config loader for the UDP endpoints used by `adsb_decoder --multicast` and the standalone emitter |
| `acceptance/` | Suites driving the real application; `python acceptance/run_all.py` (needs a display) |
| `test_codecs.py`, `test_pseudo1090.py` | Codec round-trip tests, no display needed |
| `legacy/` | The superseded multi-process tools — see `legacy/README.md` |
| `docs/reference.md` | Every major function in the above, one line each |

See **[`docs/reference.md`](docs/reference.md)** for the per-function reference.

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

## Running

### `simulator.py` — the simulator

```bash
python simulator.py
python simulator.py --centre 28,77 --range 150
python simulator.py --format-cfg my_format.cfg
```

| Flag | Default | Purpose |
|---|---|---|
| `--centre LAT,LON` | `51.477,-0.461` | Radar site position (also changeable at runtime) |
| `--range RANGE` | `200.0` | Disc radius in nautical miles |
| `--declination DEG` | `0.0` | Magnetic declination °E; converts true track → magnetic for the readout |
| `--format-cfg PATH` | `pseudo1090.cfg` | Custom 1090 MHz message format |

**Controls**

| Action | Result |
|---|---|
| Left-click empty space | drop one waypoint (auto-creates an aircraft if none selected) |
| Click-drag empty space | freehand path — points sampled at uniform spacing along the drag |
| Drag a waypoint dot | reposition it (only works once that aircraft is selected) |
| Right-click a waypoint | delete it |
| Right-click empty canvas | toggle the hover crosshair on/off |
| Aircraft list / address / callsign fields | select and rename a track |
| IFF fields | Mode 1 (5-bit), Mode 2 and 3/A (12-bit) squawks, Mode S address |
| M1 / M2 / M3-A / MC / MS checkboxes | switch individual transponder modes on and off |
| alt / speed sliders | live update of the selected aircraft |
| loop checkbox | close the path into a loop (off = fly start→end and hold) |
| Hover | crosshair with exact lat/lon under the pointer |
| F11 / Esc | toggle / leave fullscreen (radar autoscales) |
| Reset positions (red) | rewind every aircraft to the start of its path |

The right-hand panel scrolls; it holds the aircraft editor, the **RADAR SITE**
controls, the **1090 FORMAT** selector, and the **1090 AIRTIME** log.

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
python test_codecs.py            # IFF + ADS-B codec round-trips
python test_pseudo1090.py        # custom-format codec
python acceptance/run_all.py     # drives the real application; needs a display
```

The acceptance suites step a real Tk window on a monkeypatched clock and assert
against the running program, not a mock. See `acceptance/README.md`.

---

## Network configuration

`simulator.py` uses no network. `network.cfg` supplies defaults for the parts
that do: `adsb_decoder.py --multicast`, `aircraft_emulator.py` run standalone,
and the tools in `legacy/`. Every CLI flag overrides it.

```ini
# ADS-B multicast network settings
group = 239.255.0.1
port  = 30003
iface = 127.0.0.1     # loopback — change to a LAN interface for real hardware
```

---

## UI scaling

`radar_ui.SCALE` (default `2`) sets the baseline pixel density. On top of that
the window is resizable and the radar disc, blips, labels, and fonts all scale
dynamically with the live canvas size — F11 enters fullscreen, Esc leaves it.
The side panel keeps a fixed width for readability and scrolls vertically.

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

**The ground-truth barrier** — only the physics step and `channel.py` may read
`ac.lat` / `ac.mode3a` and friends. Everything the display shows arrives as an
encoded frame that survived the channel, so the visible consequences are real
ones: plots step once per scan rather than sliding continuously, an ADS-B track
has no position until a CPR even/odd pair resolves, a silent aircraft draws no
blip, and range comes from measured round-trip delay rather than from geometry.

**Monopulse azimuth** — a reply's reported bearing is the true bearing plus a
small estimation error (≈0.1° on boresight, degrading toward the beam edges),
not the beam pointing angle. Reporting the pointing angle instead leaves an
error of up to half a beamwidth — about 300 m of cross-range at 3° and 25 nm —
that jumps from scan to scan as the PRT grid drifts against the target.
`RadarSite.monopulse = False` selects the older sliding-window behaviour.

**No threads** — everything runs on the Tk main thread in a single `_frame()`
step, which is what makes the acceptance suites able to own the clock. The
render loop is layered: a cached background (rings + lat/lon grid) and a cached
route layer are rebuilt only on view/edit changes; only the per-frame
blip/label layer is redrawn each tick.

**Per-target colours** — each track is drawn in a vivid hue chosen from a
low-discrepancy (golden-ratio) sequence, so two simultaneously visible targets
never land on near-identical colours.
