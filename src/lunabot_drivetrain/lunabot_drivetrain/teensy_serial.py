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

"""Pure serial client for the Teensy motor IO protocol."""

from __future__ import annotations

import time
from typing import Callable

from lunabot_drivetrain.teensy_protocol import (
    DriveCommand,
    Frame,
    MessageType,
    ProtocolError,
    decode_frame,
    encode_frame,
    pack_drive_command,
)


class TeensySerialClient:
    """Frame-oriented protocol client around a pyserial-like port."""

    def __init__(self, port, now: Callable[[], float] | None = None) -> None:
        self._port = port
        self._now = now or time.monotonic
        self._sequence = 0
        self._rx = bytearray()
        self.last_rx_time: float | None = None

    def send_heartbeat(self) -> None:
        """Send a heartbeat frame."""
        self._write(MessageType.HEARTBEAT, b"")

    def send_reset_faults(self) -> None:
        """Ask the Teensy to clear resettable faults."""
        self._write(MessageType.RESET_FAULTS, b"")

    def send_drive_command(self, command: DriveCommand) -> None:
        """Send one normalised drivetrain command."""
        self._write(MessageType.DRIVE_COMMAND, pack_drive_command(command))

    def read_frames(self) -> list[Frame]:
        """Read and decode all complete frames currently available."""
        try:
            waiting = getattr(self._port, "in_waiting", 0)
            chunk = self._port.read(waiting or 1)
        except OSError:
            raise
        if chunk:
            self._rx.extend(chunk)

        frames: list[Frame] = []
        while 0 in self._rx:
            index = self._rx.index(0)
            raw = bytes(self._rx[: index + 1])
            del self._rx[: index + 1]
            try:
                frames.append(decode_frame(raw))
                self.last_rx_time = self._now()
            except ProtocolError:
                continue
        return frames

    def _write(self, msg_type: MessageType, body: bytes) -> None:
        self._sequence = (self._sequence + 1) & 0xFFFF
        payload = encode_frame(
            Frame(msg_type=msg_type, sequence=self._sequence, body=body)
        )
        self._port.write(payload)
