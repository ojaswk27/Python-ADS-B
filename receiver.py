"""
Receiver — the only thing allowed to build the display picture
=============================================================
Owns the track store.  Every field in it arrives by decoding a frame that came
through channel.py; nothing here may read aircraft ground truth.

Two consequences worth stating plainly, because both are behaviour the previous
"visibility filter over ground truth" design hid:

  * range and bearing are *measured* — range from the round-trip delay, bearing
    from the beam pointing angle — so they carry quantisation and noise, and a
    track only has them where a reply actually landed.

  * an ADS-B track has no position until CPR resolves.  One position frame is
    not enough: acquisition needs an even/odd pair (or a reference position),
    so a newly-seen aircraft legitimately shows an address, a callsign and an
    altitude with no lat/lon for a second or two.

Track record shape, keyed by the 24-bit Mode S address:

    {"addr", "trk_no", "color",
     "iff":  {"m1"|"m2"|"sqwk"|"alt_ft"|"call"|"modes_addr": (value, ts), ...,
              "last_ts", "last_mode", "last_raw", "last_prt"},
     "adsb": {"lat"|"lon"|"alt_ft"|"call"|"track_deg"|"vrate_fpm": (value, ts),
              ..., "last_ts", "last_type", "last_raw"},
     "plot": {"rng_nm", "brg_deg", "ts"}}

Per-field (value, timestamp) tuples are load-bearing: a Mode 1 reply must not
keep a 30-second-old squawk on screen just because it refreshed the track.
"""

import math

import iff_protocol as iff
from aircraft_emulator import decode_frame

# DO-260B 2.2.3.2.7.1: an even/odd pair older than this cannot be trusted for
# a global CPR decode.
CPR_PAIR_MAX_AGE_S = 10.0

_NZ = 15


def _nl(lat):
    """Longitude zone count at a latitude."""
    if abs(lat) >= 87.0:
        return 1
    return int(2 * math.pi / math.acos(
        1.0 - (1.0 - math.cos(math.pi / (2 * _NZ)))
        / (math.cos(math.radians(lat)) ** 2)))


def cpr_local(cpr_lat, cpr_lon, odd, ref_lat, ref_lon):
    """Local (single-frame) CPR decode against a known nearby position.

    Valid while the true position is within ~180 nm of the reference, which a
    fixed ground receiver can always satisfy using either the site position or
    the track's own last position.
    """
    i = 1 if odd else 0
    dlat = 360.0 / (4 * _NZ - i)
    yz = cpr_lat / 131072.0
    j = math.floor(ref_lat / dlat) + math.floor(
        0.5 + (ref_lat % dlat) / dlat - yz)
    lat = dlat * (j + yz)

    nl = _nl(lat)
    ni = max(nl - i, 1)
    dlon = 360.0 / ni
    xz = cpr_lon / 131072.0
    m = math.floor(ref_lon / dlon) + math.floor(
        0.5 + (ref_lon % dlon) / dlon - xz)
    lon = dlon * (m + xz)
    if lon >= 180.0:
        lon -= 360.0
    return lat, lon


class Receiver:
    """Decodes delivered frames into tracks.  No ground-truth access."""

    def __init__(self, site, colour_fn=None):
        self.site = site
        self._colour_fn = colour_fn
        self.tracks: dict[int, dict] = {}
        self.next_track_no = 1
        # addr -> "S" (Mode S reply), "A" (ADS-B), or "C" (custom 1090 format)
        self.known_addrs: dict[int, str] = {}
        # addr -> {"even": (cpr_lat, cpr_lon, ts), "odd": (...)}
        self._cpr: dict[int, dict] = {}
        self.dirty = False
        # Custom-format frames that decoded but could not be attributed to an
        # aircraft, because the config declares no address field.  Counted
        # rather than dropped silently: it is the visible consequence of
        # spending all 88 bits without reserving room for an address.
        self.unattributed = 0
        # Frames that arrived on 1090 but did not decode under the active
        # format — a standard ADS-B frame while in custom mode, or vice versa.
        self.undecodable = 0

    # ── store ────────────────────────────────────────────────────────────────

    def track_for(self, addr):
        trk = self.tracks.get(addr)
        if trk is None:
            trk = self.tracks[addr] = {
                "addr":   addr,
                "trk_no": self.next_track_no,
                "color":  self._colour_fn(addr) if self._colour_fn else None,
                "iff":    {},
                "adsb":   {},
                "pseudo": {},
                "plot":   {},
            }
            self.next_track_no += 1
            self.dirty = True
        return trk

    def forget(self, addr):
        self.tracks.pop(addr, None)
        self.known_addrs.pop(addr, None)
        self._cpr.pop(addr, None)
        self.dirty = True

    def prune(self, now, max_age_s):
        """Drop tracks with no recent frame from either source."""
        stale = [a for a, t in self.tracks.items()
                 if now - max(t["iff"].get("last_ts", 0),
                              t["adsb"].get("last_ts", 0),
                              t["pseudo"].get("last_ts", 0)) > max_age_s]
        for addr in stale:
            self.forget(addr)
        return stale

    # How informative each way of learning an address is.  A strict ranking
    # matters: with two sources active the label would otherwise flip back and
    # forth every frame, and anything keyed on it — the target dropdown — would
    # rebuild itself continuously.
    _PROVENANCE_RANK = {"A": 1, "C": 2, "S": 3}

    def _learn(self, addr, how):
        """Record how the address became known.  Only ever upgrades, so the
        value is stable while a track is alive."""
        prev = self.known_addrs.get(addr)
        if prev is not None and \
                self._PROVENANCE_RANK.get(prev, 0) >= self._PROVENANCE_RANK.get(how, 0):
            return False
        self.known_addrs[addr] = how
        return True

    # ── IFF receive path ─────────────────────────────────────────────────────

    def rx_iff(self, frame, t_tx, t_rx, beam_az_deg):
        """Decode one IFF reply and update its track.

        Range comes from the measured round-trip delay, bearing from the beam
        azimuth.  Neither is recomputed from geometry.  Returns the track, or
        None if the frame did not decode.
        """
        try:
            d = iff.decode_target_reply(frame)
        except ValueError:
            return None          # garbled or truncated: no plot, no fields

        mode = d["mode"]
        addr = d["icao"]
        trk = self.track_for(addr)
        di = trk["iff"]
        di["last_ts"]   = t_rx
        di["last_mode"] = mode
        di["last_raw"]  = frame
        di["last_prt"]  = d["prt_no"]

        # Measured plot.  rtt_to_range_nm subtracts the mode's turnaround delay
        # and quantises to the range cell, exactly as a real extractor does.
        rtt_us = (t_rx - t_tx) * 1e6
        rng_nm = iff.rtt_to_range_nm(rtt_us, mode)
        trk["plot"] = {"rng_nm": rng_nm, "brg_deg": beam_az_deg % 360.0,
                       "ts": t_rx}

        # Only the field this mode actually carries.
        if "mission_code" in d:
            di["m1"] = (d["mission_code"], t_rx)
        if "unit_code" in d:
            di["m2"] = (d["unit_code"], t_rx)
        if "squawk" in d:
            di["sqwk"] = (d["squawk"], t_rx)
        if "alt_ft" in d:
            di["alt_ft"] = (d["alt_ft"], t_rx)
        if "callsign" in d:
            di["call"] = (d["callsign"], t_rx)
        if "modes_addr" in d:
            di["modes_addr"] = (d["modes_addr"], t_rx)
            self._learn(d["modes_addr"], "S")

        self.dirty = True
        return trk

    # ── ADS-B receive path ───────────────────────────────────────────────────

    def rx_adsb(self, frame, t):
        """Decode one ADS-B frame and update its track.  Returns the track, or
        None if the frame did not decode."""
        try:
            d = decode_frame(frame)
        except ValueError:
            return None

        addr = int(d["icao"], 16)
        trk = self.track_for(addr)
        da = trk["adsb"]
        da["last_ts"]   = t
        da["last_type"] = d["kind"]
        da["last_raw"]  = d["raw"]
        self._learn(addr, "A")

        kind = d["kind"]
        if kind == "ident":
            da["call"] = (d["callsign"], t)
        elif kind == "velocity":
            if d["track_deg"] is not None:
                da["track_deg"] = (d["track_deg"], t)
            if d["speed_kt"] is not None:
                da["speed_kt"] = (d["speed_kt"], t)
            if d["vrate_fpm"] is not None:
                da["vrate_fpm"] = (d["vrate_fpm"], t)
        elif kind == "position":
            if d["alt_ft"] is not None:
                da["alt_ft"] = (d["alt_ft"], t)
            pos = self._resolve_cpr(addr, d, t, trk)
            if pos is not None:
                da["lat"] = (pos[0], t)
                da["lon"] = (pos[1], t)

        self.dirty = True
        return trk

    # ── Custom 1090 receive path ─────────────────────────────────────────────

    # Field *names* in the config are the user's choice, so they cannot be used
    # to work out what a value means.  The declared *source* can: a field fed
    # from ac.lat is a latitude whatever it is called.  This maps sources onto
    # the track keys the display already understands.
    _SOURCE_KEY = {
        "ac.lat":       "lat",
        "ac.lon":       "lon",
        "ac.alt_ft":    "alt_ft",
        "ac.speed_kt":  "speed_kt",
        "ac.track":     "track_deg",
        "ac.heading":   "track_deg",
        "ac.vrate_fpm": "vrate_fpm",
        "ac.callsign":  "call",
        "ac.mode3a":    "sqwk",
        "ac.mode1":     "m1",
        "ac.mode2":     "m2",
    }

    def rx_pseudo(self, frame, t, fmt):
        """Decode one frame under the custom 1090 format and update its track.

        Returns the track, or None if the frame did not decode under this
        format or could not be attributed to an aircraft.  Both outcomes are
        counted so the UI can show them rather than the frame just vanishing.
        """
        try:
            name, values = fmt.decode(frame)
        except Exception:
            self.undecodable += 1
            return None

        addr = fmt.address_of(name, values)
        if addr is None:
            self.unattributed += 1
            return None

        trk = self.track_for(addr)
        d = trk["pseudo"]
        d["last_ts"]   = t
        d["last_type"] = name
        d["last_raw"]  = frame
        # Keep the decoded values verbatim for the log and the detail pane,
        # alongside the spec so each can be formatted by its own field.
        # Underscored so nothing mistakes these for (value, ts) fields.
        d["_decoded"] = values
        d["_spec"] = fmt.messages[name]
        d["_decoded_ts"] = t
        self._learn(addr, "C")

        for fld in fmt.messages[name].fields:
            key = self._SOURCE_KEY.get(fld.source)
            if key is None:
                continue
            v = values.get(fld.name)
            if v is not None and v != "":
                d[key] = (v, t)

        self.dirty = True
        return trk

    def _resolve_cpr(self, addr, d, t, trk):
        """Turn a raw CPR frame into a position, or None while unresolved.

        Acquisition uses a global even/odd decode; once a position is known,
        each subsequent frame is decoded locally against it, which is what a
        fixed ground receiver does anyway.
        """
        st = self._cpr.setdefault(addr, {})
        st["odd" if d["odd_flag"] else "even"] = (d["cpr_lat"], d["cpr_lon"], t)

        # Already tracking: single-frame local decode against the last position.
        prev = trk["adsb"].get("lat"), trk["adsb"].get("lon")
        if prev[0] is not None and prev[1] is not None:
            return cpr_local(d["cpr_lat"], d["cpr_lon"], d["odd_flag"],
                             prev[0][0], prev[1][0])

        # Acquiring: needs a fresh even/odd pair.
        e, o = st.get("even"), st.get("odd")
        if not (e and o) or abs(e[2] - o[2]) > CPR_PAIR_MAX_AGE_S:
            return None
        return iff_cpr_global(e, o, use_odd=d["odd_flag"])


def iff_cpr_global(even, odd, use_odd=False):
    """Global CPR decode from an (cpr_lat, cpr_lon, ts) even/odd pair."""
    import adsb_decoder as dec
    return dec.cpr_resolve(even[0], even[1], odd[0], odd[1], use_odd=use_odd)
