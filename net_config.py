"""
Shared network configuration loader.

Reads network.cfg (KEY = VALUE, # comments) from the same directory as this
file.  Returns a dict covering: ADS-B multicast ('group'/'port'/'iface'),
ASTERIX CAT021 output ('asterix_host'/'asterix_port'), the external IFF
interrogation-input and reply-output channels ('iff_interrogation_*',
'iff_reply_*'), and the private legacy/aircraft_sim.py <-> legacy/iff_radar.py
link ('ac_channel_int_*', 'ac_channel_rep_*').  Missing or unreadable config
files are silently ignored — callers fall back to their own hard-coded defaults.

simulator.py has no network of its own; it reaches this module only indirectly,
through adsb_decoder and aircraft_emulator, and ignores every value here.  The
remaining consumers are adsb_decoder --multicast, aircraft_emulator run
standalone, and the superseded tools in legacy/.
"""

import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_CFG_PATH = os.path.join(_HERE, "network.cfg")

_DEFAULTS = {
    # ADS-B multicast (dump1090 stream)
    "group":                        "239.255.0.1",
    "port":                         "30003",
    "iface":                        "127.0.0.1",
    # ASTERIX CAT021 unicast (path_emulator)
    "asterix_host":                 "127.0.0.1",
    "asterix_port":                 "8600",
    # IFF interrogation input — messages that select the scanner's mode
    "iff_interrogation_host":       "239.100.100.1",
    "iff_interrogation_port":       "4001",
    "iff_interrogation_transport":  "multicast",
    "iff_interrogation_iface":      "127.0.0.1",
    # IFF reply output — the same bytes shown in the "LAST REPLY" pane
    "iff_reply_host":               "239.100.100.2",
    "iff_reply_port":               "4002",
    "iff_reply_transport":          "multicast",
    "iff_reply_iface":              "127.0.0.1",
    # Radar → Aircraft channel (per-PRT interrogation packet)
    "ac_channel_int_host":          "127.0.0.1",
    "ac_channel_int_port":          "5001",
    "ac_channel_int_transport":     "unicast",
    "ac_channel_int_iface":         "127.0.0.1",
    # Aircraft → Radar channel (per-target reply packet)
    "ac_channel_rep_host":          "127.0.0.1",
    "ac_channel_rep_port":          "5002",
    "ac_channel_rep_transport":     "unicast",
    "ac_channel_rep_iface":         "127.0.0.1",
}


def load() -> dict:
    """Return parsed network settings from network.cfg."""
    cfg = dict(_DEFAULTS)
    try:
        with open(_CFG_PATH) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, val = line.partition("=")
                cfg[key.strip()] = val.strip()
    except FileNotFoundError:
        pass
    return {
        "group":                        cfg["group"],
        "port":                         int(cfg["port"]),
        "iface":                        cfg["iface"],
        "asterix_host":                 cfg["asterix_host"],
        "asterix_port":                 int(cfg["asterix_port"]),
        "iff_interrogation_host":       cfg["iff_interrogation_host"],
        "iff_interrogation_port":       int(cfg["iff_interrogation_port"]),
        "iff_interrogation_transport":  cfg["iff_interrogation_transport"],
        "iff_interrogation_iface":      cfg["iff_interrogation_iface"],
        "iff_reply_host":               cfg["iff_reply_host"],
        "iff_reply_port":               int(cfg["iff_reply_port"]),
        "iff_reply_transport":          cfg["iff_reply_transport"],
        "iff_reply_iface":              cfg["iff_reply_iface"],
        "ac_channel_int_host":          cfg["ac_channel_int_host"],
        "ac_channel_int_port":          int(cfg["ac_channel_int_port"]),
        "ac_channel_int_transport":     cfg["ac_channel_int_transport"],
        "ac_channel_int_iface":         cfg["ac_channel_int_iface"],
        "ac_channel_rep_host":          cfg["ac_channel_rep_host"],
        "ac_channel_rep_port":          int(cfg["ac_channel_rep_port"]),
        "ac_channel_rep_transport":     cfg["ac_channel_rep_transport"],
        "ac_channel_rep_iface":         cfg["ac_channel_rep_iface"],
    }
