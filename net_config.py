"""
Shared network configuration loader.

Reads network.cfg (KEY = VALUE, # comments) from the same directory as this
file.  Returns a dict with keys 'group', 'port', 'iface' (ADS-B multicast) and
'asterix_host', 'asterix_port' (CAT021 / radar-position unicast output).
Missing or unreadable config files are silently ignored — callers fall back to
their own hard-coded defaults.
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
    }
