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
    "group":        "239.255.0.1",
    "port":         "30003",
    "iface":        "127.0.0.1",
    "asterix_host": "127.0.0.1",
    "asterix_port": "8600",
}


def load() -> dict:
    """Return parsed network settings from network.cfg.

    Keys: 'group' (str), 'port' (int), 'iface' (str),
          'asterix_host' (str), 'asterix_port' (int).
    """
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
        "group":        cfg["group"],
        "port":         int(cfg["port"]),
        "iface":        cfg["iface"],
        "asterix_host": cfg["asterix_host"],
        "asterix_port": int(cfg["asterix_port"]),
    }
