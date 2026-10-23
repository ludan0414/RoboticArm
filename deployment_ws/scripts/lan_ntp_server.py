#!/usr/bin/env python3
"""Serve this host's clock over SNTP on the robot LAN.

chronyd on Ubuntu refuses to start without real root, even with -x. This
process is the laptop-side time source so Orin chrony can follow the
ThinkPad. It does not step the laptop clock; systemd-timesyncd still owns
that.
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

NTP_DELTA = 2208988800  # seconds between 1900-01-01 and 1970-01-01


def _now_ntp() -> tuple[int, int]:
    t = time.time() + NTP_DELTA
    sec = int(t)
    frac = int((t - sec) * (1 << 32)) & 0xFFFFFFFF
    return sec & 0xFFFFFFFF, frac


def _pack_ts(sec: int, frac: int) -> bytes:
    return struct.pack("!II", sec, frac)


def _reply(request: bytes) -> bytes:
    if len(request) < 48:
        return b""
    recv_sec, recv_frac = _now_ntp()
    originate = request[40:48]
    # LI=0, VN=4, Mode=4 (server); stratum 10; poll=6; precision ~1 us
    header = bytes((0x24, 10, 6, (-20) & 0xFF))
    root_delay = struct.pack("!I", 0)
    # 16.16 seconds. 1 ms keeps chrony from advertising a 1 s error bar.
    root_disp = struct.pack("!I", int(0.001 * 65536))
    refid = b"LOCL"
    ref = _pack_ts(*_now_ntp())
    recv = _pack_ts(recv_sec, recv_frac)
    tx = _pack_ts(*_now_ntp())
    return header + root_delay + root_disp + refid + ref + originate + recv + tx


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.0.11")
    parser.add_argument("--port", type=int, default=3232)
    args = parser.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    print(f"lan_ntp_server listening on {args.bind}:{args.port}", flush=True)
    while True:
        data, addr = sock.recvfrom(1024)
        reply = _reply(data)
        if reply:
            sock.sendto(reply, addr)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
    except OSError as exc:
        print(f"lan_ntp_server: {exc}", file=sys.stderr)
        raise SystemExit(1)
