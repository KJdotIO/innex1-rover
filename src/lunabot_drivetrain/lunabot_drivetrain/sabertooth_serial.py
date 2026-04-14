# Copyright 2026 Leicester Lunabotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sabertooth 2x32 Packetized Serial protocol driver."""

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
    """Convert two throttle values [-1.0, 1.0] to serial bytes."""
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
