"""
Small helpers for opening UDP sockets in multicast or unicast mode from a cfg
triple (host, port, transport).

open_recv(host, port, transport, iface) → socket bound to receive.
open_send(host, port, transport, iface) → (socket, (host, port)) tuple; caller
    does sock.sendto(data, addr).

For transport == "multicast", host is the group address to join / send to.
For transport == "unicast", host is the peer address for send (recv binds on
all interfaces).  iface is the local NIC address used for multicast egress /
join.
"""

import socket
import struct


def _is_multicast(transport: str) -> bool:
    return transport.strip().lower() == "multicast"


def open_recv(host: str, port: int, transport: str, iface: str = "0.0.0.0"):
    """Open a UDP socket ready for recvfrom()."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if _is_multicast(transport):
        s.bind(("", port))
        mreq = struct.pack("4s4s",
                           socket.inet_aton(host), socket.inet_aton(iface))
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    else:
        # Unicast: bind on iface (or 0.0.0.0) and accept datagrams for us.
        s.bind((iface if iface else "", port))
    return s


def open_send(host: str, port: int, transport: str, iface: str = "127.0.0.1"):
    """Open a UDP socket ready for sendto(), plus the destination tuple."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    if _is_multicast(transport):
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                     socket.inet_aton(iface))
    return s, (host, port)


def leave_group(sock, host: str, transport: str, iface: str = "127.0.0.1"):
    """Reverse of the IP_ADD_MEMBERSHIP done in open_recv (multicast only).
    Safe to call on a unicast socket — no-op."""
    if not _is_multicast(transport):
        return
    try:
        mreq = struct.pack("4s4s",
                           socket.inet_aton(host), socket.inet_aton(iface))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
    except OSError:
        pass
