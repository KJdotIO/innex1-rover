"""Sabertooth 2x32 Packetized Serial protocol driver.

Encodes throttle commands as Packetized Serial bytes for one Sabertooth
controller.  Each controller drives two motors (M1 and M2).

Packetized Serial format (per command):
    [address, command, data, checksum]
    checksum = (address + command + data) & 0x7F

Command bytes (forward/reverse per motor):
    0 = M1 forward  (data 0–127)
    1 = M1 reverse   (data 0–127)
    4 = M2 forward  (data 0–127)
    5 = M2 reverse   (data 0–127)

A data value of 0 with any direction command is a stop for that motor.
"""

from __future__ import annotations

import struct
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import serial


_CMD_M1_FORWARD = 0
_CMD_M1_REVERSE = 1
_CMD_M2_FORWARD = 4
_CMD_M2_REVERSE = 5


def _pack_command(address: int, command: int, data: int) -> bytes:
    """Build a four-byte Packetized Serial frame."""
    if not 0 <= address <= 255:
        raise ValueError(f"address out of range: {address}")
    if not 0 <= data <= 127:
        raise ValueError(f"data out of range: {data}")
    checksum = (address + command + data) & 0x7F
    return struct.pack("BBBB", address, command, data, checksum)


def throttle_to_bytes(
    address: int, m1_throttle: float, m2_throttle: float
) -> bytes:
    """Convert two throttle values [-1.0, 1.0] to serial bytes.

    Returns the concatenated bytes for both motor commands (8 bytes total).
    Clamps input to [-1.0, 1.0].
    """
    frames = bytearray()
    for throttle, fwd_cmd, rev_cmd in [
        (m1_throttle, _CMD_M1_FORWARD, _CMD_M1_REVERSE),
        (m2_throttle, _CMD_M2_FORWARD, _CMD_M2_REVERSE),
    ]:
        clamped = max(-1.0, min(1.0, throttle))
        data = int(abs(clamped) * 127)
        data = min(data, 127)
        cmd = fwd_cmd if clamped >= 0.0 else rev_cmd
        frames.extend(_pack_command(address, cmd, data))
    return bytes(frames)


def send_stop(port: serial.Serial, address: int) -> None:
    """Send a full stop to both motors on the given controller."""
    payload = throttle_to_bytes(address, 0.0, 0.0)
    port.write(payload)


def send_throttle(
    port: serial.Serial,
    address: int,
    m1_throttle: float,
    m2_throttle: float,
) -> None:
    """Send throttle commands to both motors on the given controller."""
    payload = throttle_to_bytes(address, m1_throttle, m2_throttle)
    port.write(payload)
