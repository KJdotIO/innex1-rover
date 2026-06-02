#!/usr/bin/env python3
"""Stream simple drivetrain commands to the Teensy firmware.

Uses only the Python standard library so it works on the lab Mac without
installing pyserial.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time


def open_serial(path: str, baud: int) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    baud_const = getattr(termios, f"B{baud}")
    attrs[4] = baud_const
    attrs[5] = baud_const
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def write_line(fd: int, line: str) -> None:
    os.write(fd, line.encode("ascii") + b"\n")


def read_available(fd: int, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0.05)
        if fd not in readable:
            continue
        chunk = os.read(fd, 4096)
        if chunk:
            sys.stdout.write(chunk.decode(errors="replace"))
            sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/cu.usbmodem198521001")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--left", type=int, default=80)
    parser.add_argument("--right", type=int, default=0)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--estop-after", type=float)
    parser.add_argument("--no-reset", action="store_true")
    args = parser.parse_args()

    if args.rate <= 0:
        raise SystemExit("--rate must be positive")
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")

    fd = open_serial(args.port, args.baud)
    period = 1.0 / args.rate
    start = time.monotonic()
    estop_sent = False

    try:
        time.sleep(0.2)
        if not args.no_reset:
            write_line(fd, "R")
            write_line(fd, "Z")
        read_available(fd, 0.3)

        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= args.duration:
                break
            if (
                args.estop_after is not None
                and not estop_sent
                and elapsed >= args.estop_after
            ):
                write_line(fd, "E")
                estop_sent = True
            elif not estop_sent:
                write_line(fd, f"V {args.left} {args.right}")
            read_available(fd, min(period, 0.2))

        write_line(fd, "X")
        read_available(fd, 0.5)
    finally:
        os.close(fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
