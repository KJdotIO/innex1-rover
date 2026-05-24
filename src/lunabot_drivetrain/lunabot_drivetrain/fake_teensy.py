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

"""Fake Teensy motor IO endpoint for bridge testing."""

from __future__ import annotations

import argparse
import os
import pty
import sys
import time
import tty

from lunabot_drivetrain.teensy_protocol import (
    DriveCommand,
    DrivetrainTelemetry,
    Frame,
    MessageType,
    ProtocolError,
    TeensyStatus,
    decode_frame,
    encode_frame,
    pack_drivetrain_telemetry,
    pack_status,
    unpack_drive_command,
)


class FakeTeensy:
    """Small serial-loop simulator for the Teensy protocol."""

    def __init__(self, fd: int, telemetry_hz: float = 20.0) -> None:
        self._fd = fd
        self._sequence = 0
        self._rx = bytearray()
        self._last_command = DriveCommand(
            left=0.0,
            right=0.0,
            enabled=False,
            estop_active=False,
            motion_inhibited=False,
        )
        self._ticks = [0, 0, 0, 0]
        self._period = 1.0 / telemetry_hz
        os.set_blocking(self._fd, False)

    def run(self) -> None:
        """Serve fake status and telemetry until interrupted."""
        next_publish = time.monotonic()
        while True:
            self._read_available()
            now = time.monotonic()
            if now >= next_publish:
                self._publish()
                next_publish = now + self._period
            time.sleep(0.002)

    def _read_available(self) -> None:
        try:
            chunk = os.read(self._fd, 256)
        except BlockingIOError:
            return
        except OSError:
            return
        self._rx.extend(chunk)
        while 0 in self._rx:
            index = self._rx.index(0)
            raw = bytes(self._rx[: index + 1])
            del self._rx[: index + 1]
            try:
                frame = decode_frame(raw)
            except ProtocolError:
                continue
            if frame.msg_type == MessageType.DRIVE_COMMAND:
                self._last_command = unpack_drive_command(frame.body)

    def _publish(self) -> None:
        left = self._last_command.left if self._last_command.enabled else 0.0
        right = self._last_command.right if self._last_command.enabled else 0.0
        self._ticks[0] += int(left * 18)
        self._ticks[2] += int(left * 18)
        self._ticks[1] += int(right * 18)
        self._ticks[3] += int(right * 18)

        state = 2 if self._last_command.enabled and (left or right) else 1
        status = TeensyStatus(
            state=state,
            fault_code=0,
            estop_active=self._last_command.estop_active,
            motion_inhibited=self._last_command.motion_inhibited,
            controller_online=(True, True),
        )
        telemetry = DrivetrainTelemetry(
            encoder_ticks=tuple(self._ticks),
            wheel_velocity_rps=(left, right, left, right),
            fault_code=0,
            estop_active=self._last_command.estop_active,
            motion_inhibited=self._last_command.motion_inhibited,
            controller_online=(True, True),
        )
        self._write(MessageType.STATUS, pack_status(status))
        self._write(
            MessageType.DRIVETRAIN_TELEMETRY,
            pack_drivetrain_telemetry(telemetry),
        )

    def _write(self, msg_type: MessageType, body: bytes) -> None:
        self._sequence = (self._sequence + 1) & 0xFFFF
        os.write(
            self._fd,
            encode_frame(Frame(msg_type=msg_type, sequence=self._sequence, body=body)),
        )


def main(argv: list[str] | None = None) -> None:
    """Run a fake Teensy and print the PTY path for the bridge."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--telemetry-hz", type=float, default=20.0)
    args = parser.parse_args(argv)

    master, slave = pty.openpty()
    tty.setraw(slave)
    print(os.ttyname(slave), flush=True)
    try:
        FakeTeensy(master, telemetry_hz=args.telemetry_hz).run()
    except KeyboardInterrupt:
        pass
    finally:
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main(sys.argv[1:])
