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

"""Tests for the pure Teensy serial client."""

from lunabot_drivetrain.teensy_protocol import (
    DriveCommand,
    Frame,
    MessageType,
    decode_frame,
    encode_frame,
    unpack_drive_command,
)
from lunabot_drivetrain.teensy_serial import TeensySerialClient


class FakePort:
    def __init__(self) -> None:
        self.writes = bytearray()
        self.reads = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self.reads)

    def write(self, data: bytes) -> int:
        self.writes.extend(data)
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self.reads[:size])
        del self.reads[:size]
        return chunk


def test_send_drive_command_writes_framed_payload():
    port = FakePort()
    client = TeensySerialClient(port)

    client.send_drive_command(
        DriveCommand(
            left=0.25,
            right=-0.5,
            enabled=True,
            estop_active=False,
            motion_inhibited=False,
        )
    )

    frame = decode_frame(bytes(port.writes))
    command = unpack_drive_command(frame.body)
    assert frame.msg_type == MessageType.DRIVE_COMMAND
    assert command.left == 0.25
    assert command.right == -0.5
    assert command.enabled is True


def test_read_frames_discards_corrupt_frame_and_keeps_good_frame():
    now = [10.0]
    port = FakePort()
    client = TeensySerialClient(port, now=lambda: now[0])
    bad = bytearray(encode_frame(Frame(MessageType.HEARTBEAT, 1, b"")))
    bad[2] ^= 0x44
    good = encode_frame(Frame(MessageType.STATUS, 2, b"\x01\x00\x00\x00\x03"))
    port.reads.extend(bytes(bad) + good)

    frames = client.read_frames()

    assert len(frames) == 1
    assert frames[0].msg_type == MessageType.STATUS
    assert frames[0].sequence == 2
    assert client.last_rx_time == 10.0
