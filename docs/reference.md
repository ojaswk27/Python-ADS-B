# Code reference

Major functions in `simulator.py` and the modules it depends on. One line each.
Superseded programs in `legacy/` are not covered.

Data flows one way:

```
SimAircraft.step()        physics, ground truth
  -> .iff_reply() / .adsb_frame() / .pseudo_frame()      encode
    -> channel.deliver_iff() / .deliver_adsb()           lose, delay, garble
      -> Receiver.rx_iff() / .rx_adsb() / .rx_pseudo()   decode into tracks
        -> CombinedApp._draw() / ._table_row()           display
```

Only the physics step and `channel.py` may read `ac.lat`, `ac.mode3a` and the
rest of ground truth. Everything past the channel sees frames only.

---

## simulator.py

The application. Holds the airspace, the interrogator, and the display.

### Module helpers

| Function | Does |
|---|---|
| `_fld(d, key, now, limit)` | Read a `(value, timestamp)` field, returning `None` once it is older than `limit`, so stale fields dash out independently. |
| `_fld_age(d, key, now)` | Age in seconds of such a field, or `None` if never set. |
| `_rx_field(trk, key, now, limit)` | Same read across all receive records (`adsb`, `pseudo`), returning `(value, ts, which)` for the freshest live one. |
| `fmt_alt(ft)` | Altitude as `FL350` above the transition level, plain feet below it. |

### class SimAircraft

One simulated aircraft: a waypoint path, a transponder, and an emitter.

| Method | Does |
|---|---|
| `step(dt)` | Advance along the path by `dt`, then update heading and altitude. |
| `course()` | Raw bearing of the segment being flown. Steps discontinuously at each waypoint, so it is not what gets reported. |
| `heading()` | The reported heading. Equals `course()` when `turn_rate_deg_s` is 0, otherwise lags it by at most that many degrees per second. |
| `_step_heading(dt)` | Turn toward `course()` at the configured rate. Does nothing without a path, so the 0.0 placeholder from `course()` is never latched as "north". |
| `_step_altitude(dt)` | Climb or descend toward the target altitude and report the vertical rate actually achieved. |
| `has_xpdr(mode_code)` | Whether this aircraft's transponder answers that IFF mode. |
| `iff_reply(mode, prt_no, qnh_hpa)` | Encode the reply to one interrogation, or `None` with no transponder for that mode and no position. |
| `adsb_frame(now, qnh_hpa)` | Next due ADS-B squitter as `(label, hex)`. Each message type has its own scheduler at DO-260B rates. |
| `pseudo_frame(now, fmt)` | Same, for the custom 1090 format, with periods taken from the config. |

### class CombinedApp

The Tk window and the simulation loop.

**Loop**

| Method | Does |
|---|---|
| `_loop()` | Tk timer callback: calls `_frame()`, then reschedules. |
| `_frame()` | One simulation frame. Split out from `_loop` so tests can own the clock. |
| `_draw()` | Redraw the canvas in three cached layers: background, routes, then blips. |

**Interrogation**

| Method | Does |
|---|---|
| `_azimuth_now()` / `_azimuth_at(t, rpm)` | Where the beam is pointing now, or at time `t`. |
| `_current_mode()` | The IFF mode this scan is interrogating in. |
| `_run_interrogation(now, dt)` | Fire every PRT that fell inside this frame. |
| `_one_interrogation(az, t)` | One PRT: collect replies from aircraft inside the beam, apply garble, hand the survivors to the receiver. |

**Display**

| Method | Does |
|---|---|
| `_reported_pos(trk, now)` | Where a track was last reported to be, as `(lat, lon, source)`. Ordered by accuracy, not freshness — a current reported position always beats a measured plot, which is what stops the symbol alternating between sources once per scan. |
| `_table_row()` / `_flush_table()` | Build and write the track table. |
| `_refresh_detail()` | Decoded readout of the selected track: every accumulated IFF field, the ADS-B state, and the measured plot, each with its own age. |
| `_refresh_target_menu()` | Rebuild the selective-target dropdown only when its entries actually change, and never while it is posted. |
| `_view_sig()` | Centre, range, and canvas size, hashed — the cached layers rebuild when it changes. |

**Radar site**

| Method | Does |
|---|---|
| `_apply_site(_ev)` | Move or resize the radar. Clears every track: a stored plot is a range and bearing measured from the *old* position. |
| `_centre_on_selected()` | Put the radar under the selected aircraft. |
| `_refresh_site()` | Push the current site values back into the entry boxes. |

**Custom 1090**

| Method | Does |
|---|---|
| `_apply_txmode(_val)` | Switch between `standard`, `custom`, and `both`. Refuses `custom` with no format loaded. |
| `_reload_fmt()` | Re-read the config without restarting. A bad file leaves the previous format in place. |
| `_log_1090(kind, label, frame, decoded, lost)` | Record one transmission in the airtime log. Append-only and capped. |
| `_flush_log()` | Render the log pane. |

---

## channel.py

Propagation. The only module on the receive path allowed to read ground truth,
and only for geometry. Every `deliver_*` returns `None` when the frame is lost.

| Function | Does |
|---|---|
| `RadarSite` | Dataclass: site position, antenna height, max ranges, loss probabilities, azimuth accuracy. |
| `radio_horizon_nm(h_ant_ft, h_ac_ft)` | Line-of-sight range for two heights, `1.23 * (√h_ant + √h_ac)`. |
| `visible(ac, site, max_range_nm)` | `(range, True)` if the aircraft is inside both the range limit and the horizon. |
| `deliver_iff(ac, site, mode, t_tx, prt_no, ...)` | Carry one IFF reply back. Returns `(frame, t_rx)` where `t_rx` includes the true round-trip delay — the receiver's only honest source of range. |
| `deliver_adsb(ac, site, frame, t)` | Carry one 1090 MHz broadcast. No delay, no garble; just range, horizon, and per-frame loss. |
| `garbled(ranges_nm, window_nm)` | Indices of Mode A/C replies that overlap in the range gate and corrupt each other. |
| `measure_bearing(beam_az_deg, site, true_brg_deg)` | The azimuth reported for one reply: true bearing plus monopulse estimation error, degrading toward the beam edges. Falls back to the beam pointing angle when `site.monopulse` is off. |

---

## receiver.py

Owns the track store. Every field in it arrives by decoding a frame; nothing
here reads ground truth.

| Function | Does |
|---|---|
| `Receiver.track_for(addr)` | Fetch or create the track for an address. |
| `Receiver.rx_iff(frame, t_tx, t_rx, beam_az_deg)` | Decode one IFF reply. Range comes from the measured round-trip delay, bearing from `measure_bearing`; neither is recomputed from geometry. |
| `Receiver.rx_adsb(frame, t)` | Decode one ADS-B frame into its track. |
| `Receiver.rx_pseudo(frame, t, fmt)` | Same under the custom 1090 format. Counts frames that fail to decode or cannot be attributed. |
| `Receiver._resolve_cpr(addr, d, t, trk)` | Turn a raw CPR frame into a position, or `None` while unresolved: global even/odd decode to acquire, local decode against the last fix thereafter. |
| `Receiver._learn(addr, how)` | Record how the address became known (`A` < `C` < `S`). Only ever upgrades, so it is stable while the track lives. |
| `Receiver.prune(now, max_age_s)` | Drop tracks with no recent frame from any source. |
| `Receiver.forget(addr)` | Delete one track. |
| `cpr_local(cpr_lat, cpr_lon, odd, ref_lat, ref_lon)` | Single-frame CPR decode against a known nearby position, valid within ~180 nm. |
| `iff_cpr_global(even, odd, use_odd)` | Global CPR decode from a timestamped even/odd pair. |

---

## iff_protocol.py

IFF reply codec, Mode C altitude, Mode 1 codes, and the shared geometry helpers.

| Function | Does |
|---|---|
| `encode_target_reply(icao_int, mode, prt_no, src_lat, src_lon, **kw)` | Build one per-target reply. The required keyword depends on the mode. |
| `decode_target_reply(pkt)` | Decode one per-target reply. Only the keys that mode actually carries are present. Raises on a truncated or malformed frame. |
| `build_reply(prt_no, azimuth_deg, mode, targets)` | Pack a reply block, truncated to `MAX_TARGETS`. |
| `decode_reply(reply)` | Unpack a reply block into a header dict plus per-target dicts. |
| `pack_interrogation(...)` / `unpack_interrogation(pkt)` | The interrogation packet: mode, PRT number, site position, beam azimuth and width. |
| `encode_mode_c(altitude_ft)` / `decode_mode_c(code)` | Mode C altitude code, lossy to 25 ft. |
| `encode_mode_c_alt(geo_alt_ft, qnh_hpa)` | Geometric altitude to the *pressure* altitude Mode C actually reports — always referenced to 1013.25 hPa, never local QNH. |
| `mode1_to_wire(v)` / `mode1_from_wire(bits)` | Mode 1 display form (`0o00`–`0o73`) to and from the packed 5-bit wire form. These are two different representations; masking one as the other silently corrupts codes above `0o33`. |
| `pack_callsign_bds20(callsign)` / `unpack_callsign_bds20(reg6)` | 8 characters at 6 bits each, as a real Mode S identification reply carries them. |
| `range_to_rtt_us(rng_nm, mode)` / `rtt_to_range_nm(rtt_us, mode)` | Range to round-trip delay and back: 12.3559 µs per nm plus the mode's turnaround. |
| `turnaround_us(mode)` | Transponder turnaround delay — 3 µs for Modes 1/2/3A/C, 128 µs for Mode S. |
| `bearing_range_nm(c_lat, c_lon, lat, lon)` | Bearing from true north and range in nm. |
| `angle_diff(a, b)` | Smallest signed difference `a - b`, in ±180°. |
| `emergency_label(squawk)` | Name for a special-purpose squawk, or `None`. |

---

## pseudo1090.py

Config-driven codec for a custom 1090 MHz format that replaces ADS-B on the
same physical layer. Same 112-bit frame and CRC-24; all 88 payload bits come
from the cfg.

| Function | Does |
|---|---|
| `load(path, strict)` | Read a config and return a `Format`. `strict=False` collects problems instead of raising, for `--check`. |
| `Field.encode(value)` / `.decode(raw)` | Python value to and from the unsigned integer that occupies the bits. |
| `Field.resolution()` | Smallest change the field can represent. |
| `Field.format(value)` | Human-readable rendering for the log and the decoded pane. |
| `Message.field(name)` | Look up one field by name. |
| `Format.encode(msg_name, ac)` | Build one 28-character frame: 88 payload bits packed per the config, plus CRC-24. |
| `Format.decode(frame)` | Decode a frame to `(msg_name, {field: value})`. Raises `DecodeError` on wrong length, bad CRC, or an undefined type id. |
| `Format.address_of(msg_name, values)` | The aircraft address a decoded message correlates to, or `None` if the config never declared one. |
| `Format.has_address()` / `.describe()` | Whether messages can be attributed at all; a short summary for `--check` and the UI. |
| `_parse_field(section, name, spec)` | `<offset> <width> <encoding> <source> [role=x]` to a `Field`. |
| `_validate(fmt, messages)` | Collect every problem at once, so one `--check` run fixes the file. |

---

## aircraft_emulator.py

ADS-B frame builders, shared with `simulator.py`. Also runs standalone as a
headless multicast emitter.

| Function | Does |
|---|---|
| `build_identification(icao, callsign, category)` | TC 4 Aircraft Identification. |
| `build_position(icao, lat, lon, alt_ft, odd)` | TC 11 Airborne Position, barometric altitude, CPR encoded. |
| `build_velocity(icao, speed_kt, heading_deg, vrate_fpm)` | TC 19 Airborne Velocity. Switches from subtype 1 to 2 at 1022 kt. |
| `decode_frame(raw)` | Decode any of the above. Position frames return *raw CPR*, not lat/lon — resolving it is the receiver's job. |
| `_encode_cpr(lat, lon, odd)` | Position to a 17-bit CPR pair, the reverse of `cpr_resolve`. |
| `_encode_altitude(alt_ft)` | The 12-bit AC field, Q=1 25-ft linear encoding. |
| `_sign_crc(payload_hex)` | Append CRC-24 to an 11-byte payload to make a complete frame. |
| `run_emulator(aircraft, group, port, iface, rate)` | Standalone transmit loop over UDP multicast. |

---

## adsb_decoder.py

Pure-stdlib Mode S decoder. Used by the receiver for CPR and altitude, and
runnable on its own against a file or a live feed.

| Function | Does |
|---|---|
| `decode_message(raw, fleet)` | Decode one frame and update the fleet dict in place. |
| `parse_header(msg)` | `(DF, ICAO, TC)` from a 28-character hex frame. |
| `crc_valid(msg)` | Whether the frame passes CRC-24. |
| `me_payload(msg)` | The 56-bit ME payload, bits 33–88. |
| `decode_identification(msg)` | Callsign and category from TC 1–4. |
| `decode_altitude(msg)` | Barometric altitude from TC 9–18, GNSS from TC 20–22. |
| `decode_cpr_fields(msg)` | Raw CPR latitude, longitude, and the odd/even flag. |
| `cpr_resolve(...)` | Global position from an even/odd pair, via the coprime 60/59 zone grids. |
| `decode_velocity(msg)` | Ground speed, track, and vertical rate from TC 19. |
| `Aircraft.ingest_cpr(fmt, cpr_lat, cpr_lon, alt)` | Store a CPR frame and resolve position once a valid pair exists. |
| `Aircraft.summary()` | One-line state for the console output. |

---

## radar_ui.py

Shared drawing primitives and Tk widget helpers. No simulation logic.

| Function | Does |
|---|---|
| `geom(w, h)` | `(cx, cy, r)` for the PPI on a canvas of that size. |
| `scale_for(w, h)` | Drawing scale factor for the live canvas size, `1.0` at baseline. |
| `ll_to_xy(...)` / `xy_to_ll(...)` | Between lat/lon and canvas pixels for the current view. |
| `draw_radar_frame(...)` | Disc, range rings, axis cross, cardinal letters. Static between view changes, so it is drawn under its own tag. |
| `draw_latlon_grid(...)` | WGS-84 graticule clipped to the disc. |
| `draw_blip(cv, x, y, hdg_rad, col, sf, tag)` | Filled aircraft triangle pointing along the heading. |
| `make_panel(root, side, width)` | Fixed-width side panel. |
| `make_scroll_panel(root, side, width)` | Same, but vertically scrollable. Tk has no scrollable Frame, so this is a Canvas with a Frame windowed into it; returns the inner Frame to pack into. |
| `random_color()` | A vivid hex colour, well separated from the last one issued. |
| `shade(hex_color, factor)` / `blend(hex_a, hex_b, t)` | Brightness scaling and linear interpolation. |
| `sep`, `entry_row`, `slider_row`, `flat_button` | Panel widget shorthands. |

---

## net_config.py

| Function | Does |
|---|---|
| `load()` | Read `network.cfg` and return the endpoint dict, falling back to built-in defaults. `simulator.py` has no network and ignores every value here. |
