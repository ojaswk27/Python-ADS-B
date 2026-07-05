"""
ADS-B ingest: subscribes to the multicast feed and mirrors decoded aircraft
into the airspace sim's aircraft list so the IFF scanner interrogates them
alongside simulated ones.

Design: keep a private fleet decoded via adsb_decoder.decode_message.  A
background thread listens on UDP, decodes each frame into the private fleet,
and on every message re-projects that fleet as `AdsbAircraft` wrapper objects
into the sim's live list under the sim lock.  Aircraft silent for >60 s are
dropped.
"""

import socket
import struct
import threading
import time

import adsb_decoder
import radar_ui as ui


_STALE_S = 60.0


class AdsbAircraft:
    """Adapter that makes an adsb_decoder.Aircraft look enough like a
    WaypointAircraft for the IFF scanner to interrogate it.

    Only the attributes the scanner touches are exposed.  We don't share
    `__slots__` with WaypointAircraft — duck typing is fine here.
    """

    def __init__(self, ac: "adsb_decoder.Aircraft"):
        self.icao       = ac.icao
        self.callsign   = (ac.callsign or ac.icao).strip()
        self.color      = ui.random_color()
        # No route — position comes live from ADS-B.  These attributes exist
        # only so sim code that walks every aircraft (list refresh, reset-
        # positions button, route drawer) doesn't blow up on missing fields.
        self.waypoints  = []
        self.loop       = False
        self.alt_ft     = ac.altitude or 0
        self.speed_kt   = ac.speed or 0
        self._seg       = 0
        self._seg_t     = 0.0
        self._lat       = None
        self._lon       = None
        # IFF state: only Mode S is meaningful (real transponders don't have
        # simulated squawks).  ICAO drives the Mode S address.
        self.modes_addr = int(ac.icao, 16) & 0xFFFFFF
        self.mode1  = 0
        self.mode2  = 0
        self.mode3a = 0
        self.xpdr1 = False
        self.xpdr2 = False
        self.xpdr3a = False
        self.xpdrC = False
        self.xpdrS = True

        self._decoder_ac = ac         # keep the source alive for lat/lon lookup

    _MODE_FLAG = {1: "xpdr1", 2: "xpdr2", 3: "xpdr3a", 4: "xpdrC",
                  5: "xpdrS", 6: "xpdrS"}

    def has_xpdr(self, mode_code: int) -> bool:
        return getattr(self, self._MODE_FLAG.get(mode_code, ""), False)

    def step(self, dt):
        """No-op: position for ADS-B tracks comes from received messages,
        not from waypoint interpolation.  The sim's per-frame loop calls
        step() on every aircraft, so we must accept the call."""
        return

    @property
    def lat(self):
        return self._decoder_ac.lat

    @property
    def lon(self):
        return self._decoder_ac.lon

    def heading(self):
        ac = self._decoder_ac
        return ac.track if ac.track is not None else (ac.heading or 0.0)

    def seconds_since_last_msg(self):
        """Seconds since the decoder last saw a message for this ICAO.
        Returns +inf if we've never had one (shouldn't happen on a registered
        aircraft) so callers treat it as fully faded."""
        from datetime import datetime, timezone
        if self._decoder_ac.last_seen is None:
            return float("inf")
        return (datetime.now(timezone.utc) - self._decoder_ac.last_seen).total_seconds()

    def refresh_from_decoder(self):
        """Pick up any newly-decoded fields (callsign, speed, altitude)."""
        ac = self._decoder_ac
        if ac.callsign and ac.callsign.strip():
            self.callsign = ac.callsign.strip()
        if ac.altitude is not None:
            self.alt_ft = ac.altitude
        if ac.speed is not None:
            self.speed_kt = ac.speed


class AdsbSource:
    """Background thread that decodes an ADS-B multicast/unicast stream and
    keeps a set of AdsbAircraft objects registered in the sim."""

    def __init__(self, sim, group, port, iface):
        self.sim = sim
        self._group, self._port, self._iface = group, port, iface
        self._sock = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        # Map of icao → AdsbAircraft currently registered on the sim.
        self._registered: dict[str, AdsbAircraft] = {}
        # Private decoder fleet (adsb_decoder.Aircraft objects).
        self._fleet: dict[str, "adsb_decoder.Aircraft"] = {}

    def start(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("", self._port))
            mreq = struct.pack("4s4s",
                               socket.inet_aton(self._group),
                               socket.inet_aton(self._iface))
            s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            s.settimeout(0.5)
            self._sock = s
        except OSError as e:
            print(f"[adsb_source] receive socket failed: {e}")
            return
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass

    def _rx_loop(self):
        buf = ""
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                self._prune_stale()
                continue
            except OSError:
                return
            buf += data.decode("ascii", errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    adsb_decoder.decode_message(line, self._fleet)
                except Exception:
                    continue
                self._sync_fleet()

    def _sync_fleet(self):
        """Reflect the decoder fleet into the sim's aircraft list."""
        with self.sim._lock:
            for icao, dac in self._fleet.items():
                # Skip aircraft that don't yet have a position (CPR pair not
                # resolved) — no point interrogating something we can't draw.
                if dac.lat is None or dac.lon is None:
                    continue
                wrap = self._registered.get(icao)
                if wrap is None:
                    wrap = AdsbAircraft(dac)
                    self._registered[icao] = wrap
                    self.sim._aircraft.append(wrap)
                    self.sim._dirty = True
                else:
                    wrap.refresh_from_decoder()

    def _prune_stale(self):
        """Drop aircraft that haven't been heard from in _STALE_S seconds."""
        from datetime import datetime, timezone
        now = datetime.now(timezone.utc)
        stale = []
        for icao, dac in list(self._fleet.items()):
            if dac.last_seen is None:
                continue
            if (now - dac.last_seen).total_seconds() > _STALE_S:
                stale.append(icao)
        if not stale:
            return
        with self.sim._lock:
            for icao in stale:
                self._fleet.pop(icao, None)
                wrap = self._registered.pop(icao, None)
                if wrap is not None:
                    try:
                        self.sim._aircraft.remove(wrap)
                    except ValueError:
                        pass
                    if self.sim._selected is wrap:
                        self.sim._selected = None
            self.sim._dirty = True
