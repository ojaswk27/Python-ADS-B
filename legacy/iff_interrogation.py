"""
Incoming IFF interrogation message parser + UDP receiver.
"""

import os
import socket
import struct
import sys
import threading
from dataclasses import dataclass
from typing import Callable, Optional

import udp_endpoints as udp


def _load_magic():
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".formats")
    if d not in sys.path:
        sys.path.insert(0, d)
    try:
        import magic  # type: ignore
        return magic.IFF_INTERROGATION_ID, magic.IFF_INTERROGATION_MSG_LEN
    except Exception:
        return 0, 0

ID_MAGIC, MSG_LEN = _load_magic()
_STRUCT      = struct.Struct(">HHB14sB5sB")


@dataclass
class Interrogation:
    """A parsed incoming interrogation message."""
    mode:          int
    mode_s_data:   bytes            # 14 bytes as received
    mode_s_long:   bool
    modes_addr:    Optional[int]    # first 3 bytes of mode_s_data (or None if zero)


def parse(pkt: bytes) -> Optional[Interrogation]:
    """Parse an incoming packet.  Returns None on any format error."""
    if len(pkt) < MSG_LEN:
        return None
    try:
        magic, length, mode, ms_data, ms_long, _m5, _crypto = _STRUCT.unpack(pkt[:MSG_LEN])
    except struct.error:
        return None
    if magic != ID_MAGIC:
        return None
    # length field: use the value if it looks sane, but don't reject a packet
    # that carries trailing bytes — some senders pad.
    if length not in (MSG_LEN, 0):
        pass
    addr_raw = int.from_bytes(ms_data[:3], "big")
    addr = addr_raw if addr_raw != 0 else None
    return Interrogation(
        mode        = mode,
        mode_s_data = bytes(ms_data),
        mode_s_long = bool(ms_long),
        modes_addr  = addr,
    )


# ── UDP receiver ──────────────────────────────────────────────────────────────

class Receiver:
    """Background thread that listens for interrogation messages and hands
    each successfully-parsed one to the callback.  Silent on parse errors."""

    def __init__(self, host: str, port: int, transport: str, iface: str,
                 on_message: Callable[[Interrogation], None]):
        self._host, self._port = host, port
        self._transport, self._iface = transport, iface
        self._on_message = on_message
        self._sock = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        try:
            self._sock = udp.open_recv(self._host, self._port,
                                       self._transport, self._iface)
            self._sock.settimeout(0.5)
        except OSError as e:
            print(f"[iff_interrogation] receive socket failed: {e}")
            return
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._sock is not None:
            try:
                udp.leave_group(self._sock, self._host,
                                self._transport, self._iface)
                self._sock.close()
            except OSError:
                pass

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                # Socket closed during shutdown, or transient error.
                if self._stop.is_set():
                    return
                continue
            msg = parse(data)
            if msg is not None:
                try:
                    self._on_message(msg)
                except Exception:
                    pass


# Small helper for building test packets — used by unit tests.
def build(mode: int, mode_s_data: bytes = b"\x00" * 14, mode_s_long: bool = True,
          mode_5_data: bytes = b"\x00" * 5, crypto_op: int = 0) -> bytes:
    if len(mode_s_data) < 14:
        mode_s_data = mode_s_data + b"\x00" * (14 - len(mode_s_data))
    return _STRUCT.pack(ID_MAGIC, MSG_LEN, mode & 0xFF, mode_s_data[:14],
                        1 if mode_s_long else 0, mode_5_data[:5], crypto_op & 0xFF)
