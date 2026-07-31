# legacy/

Superseded programs, kept for reference. **Nothing here is needed to run
`simulator.py`** — the current simulator's entire dependency closure is

```
simulator.py
  ├── channel.py          ── iff_protocol.py
  ├── receiver.py         ── iff_protocol.py, aircraft_emulator.py, adsb_decoder.py
  ├── pseudo1090.py       ── aircraft_emulator.py, adsb_decoder.py
  ├── radar_ui.py
  ├── aircraft_emulator.py ── net_config.py (→ network.cfg)
  └── adsb_decoder.py     ── net_config.py
```

and it touches none of these files.

## Why they were superseded

The old design was **multi-process, over UDP**: an emulator transmitted on a
multicast group, a separate display received and decoded. `simulator.py`
replaced that with one process holding an emitter → channel → receiver
pipeline, so the propagation model (range, radio horizon, frame loss,
round-trip delay, garble) is explicit and testable instead of being whatever
the loopback network happened to do.

| File | Was | Replaced by |
|---|---|---|
| `aircraft_sim.py` | GUI airspace emulator, transmitted over UDP | the emitter half of `simulator.py` |
| `iff_radar.py` | GUI IFF/SSR interrogator + PPI, received over UDP | the interrogator/display half of `simulator.py` |
| `adsb_source.py` | RX thread feeding `aircraft_sim.py` | `receiver.py` |
| `iff_interrogation.py` | wire format for the UDP interrogation link | `channel.py` (no wire link any more) |
| `udp_endpoints.py` | multicast/unicast socket helpers | — (single process) |
| `path_emulator.py` | GUI path editor + ADS-B/CAT021/radar-position TX | the waypoint editing in `simulator.py` |
| `radar_display.py` | GUI ADS-B-only PPI receiver | the display half of `simulator.py` |
| `cat21.py` | ASTERIX CAT021 encoder, used only by `path_emulator.py` | — (not carried forward) |
| `interactive_emulator.py` | earlier GUI emulator | `path_emulator.py`, then `simulator.py` |
| `ppi_display.py` | curses text-mode PPI | `radar_display.py`, then `simulator.py` |
| `test_velocity.py` | unittest suite for `build_velocity`, decoded with pyModeS | `test_codecs.py`, which round-trips velocity (plus the other codecs) through this project's own decoder. Finer-grained than `test_codecs.py` on velocity alone, so it is kept and still passes — it is just no longer in `run_all.py` |

## Running them

They still work. Each one that imports a shared module (`radar_ui`,
`net_config`, `adsb_decoder`, `aircraft_emulator`) has a three-line shim at the
top of its import block putting the repo root on `sys.path`, so run them by
path from anywhere:

```bash
python legacy/aircraft_sim.py --help
python legacy/iff_radar.py --centre 51.5,-0.5 --range 150
python -m unittest discover -s legacy -p "test_velocity.py"
```

`aircraft_sim.py` and `iff_radar.py` are a matched pair — they talk to each
other over the `ac_channel_*` endpoints in `network.cfg`, so run both.
