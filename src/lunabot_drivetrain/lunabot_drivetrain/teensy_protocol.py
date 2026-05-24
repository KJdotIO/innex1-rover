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

"""Framed USB serial protocol for the Teensy motor IO controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

PROTOCOL_VERSION = 1
MAX_BODY_BYTES = 96


class MessageType(IntEnum):
    """Protocol message identifiers."""

    HEARTBEAT = 1
    DRIVE_COMMAND = 2
    CONTROL_FLAGS = 3
    RESET_FAULTS = 4
    STATUS = 0x80
    DRIVETRAIN_TELEMETRY = 0x81
    FAULT_EVENT = 0x82


class ProtocolError(ValueError):
    """Raised when an encoded frame is invalid."""


@dataclass(frozen=True)
class Frame:
    """Decoded Teensy protocol frame."""

    msg_type: MessageType
    sequence: int
    body: bytes = b""
    version: int = PROTOCOL_VERSION


@dataclass(frozen=True)
class DriveCommand:
    """Normalised per-side drivetrain command."""

    left: float
    right: float
    enabled: bool
    estop_active: bool
    motion_inhibited: bool


@dataclass(frozen=True)
class TeensyStatus:
    """Controller status reported by the Teensy."""

    state: int
    fault_code: int
    estop_active: bool
    motion_inhibited: bool
    controller_online: tuple[bool, bool]


@dataclass(frozen=True)
class DrivetrainTelemetry:
    """Drivetrain telemetry reported by the Teensy."""

    encoder_ticks: tuple[int, int, int, int]
    wheel_velocity_rps: tuple[float, float, float, float]
    fault_code: int
    estop_active: bool
    motion_inhibited: bool
    controller_online: tuple[bool, bool]


def crc16_ccitt(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE for *data*."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode *data* and append a zero frame delimiter."""
    out = bytearray()
    code_index = 0
    code = 1
    out.append(0)

    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue

        out.append(byte)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1

    out[code_index] = code
    out.append(0)
    return bytes(out)


def cobs_decode(frame: bytes) -> bytes:
    """Decode one zero-delimited COBS frame."""
    if not frame or frame[-1] != 0:
        raise ProtocolError("COBS frame must end with zero delimiter")

    out = bytearray()
    index = 0
    end = len(frame) - 1
    while index < end:
        code = frame[index]
        index += 1
        if code == 0 or index + code - 1 > end:
            raise ProtocolError("invalid COBS code")
        out.extend(frame[index:index + code - 1])
        index += code - 1
        if code < 0xFF and index < end:
            out.append(0)
    return bytes(out)


def encode_frame(frame: Frame) -> bytes:
    """Encode a protocol frame for serial transmission."""
    if frame.version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {frame.version}")
    if len(frame.body) > MAX_BODY_BYTES:
        raise ProtocolError(f"frame body too large: {len(frame.body)}")
    payload = struct.pack("<BBH", frame.version, int(frame.msg_type), frame.sequence)
    payload += frame.body
    payload += struct.pack("<H", crc16_ccitt(payload))
    return cobs_encode(payload)


def decode_frame(frame: bytes) -> Frame:
    """Decode and validate one protocol frame."""
    payload = cobs_decode(frame)
    if len(payload) < 6:
        raise ProtocolError("frame too short")
    if len(payload) > 4 + MAX_BODY_BYTES + 2:
        raise ProtocolError("frame too large")

    expected_crc = struct.unpack_from("<H", payload, len(payload) - 2)[0]
    body_without_crc = payload[:-2]
    actual_crc = crc16_ccitt(body_without_crc)
    if expected_crc != actual_crc:
        raise ProtocolError("crc mismatch")

    version, raw_type, sequence = struct.unpack_from("<BBH", body_without_crc)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    try:
        msg_type = MessageType(raw_type)
    except ValueError as exc:
        raise ProtocolError(f"unknown message type {raw_type}") from exc
    return Frame(msg_type=msg_type, sequence=sequence, body=body_without_crc[4:])


def _scale_throttle(value: float) -> int:
    clamped = max(-1.0, min(1.0, float(value)))
    return int(round(clamped * 10000))


def pack_drive_command(command: DriveCommand) -> bytes:
    """Pack a drivetrain command body."""
    flags = 0
    flags |= int(command.enabled) << 0
    flags |= int(command.estop_active) << 1
    flags |= int(command.motion_inhibited) << 2
    return struct.pack(
        "<hhB",
        _scale_throttle(command.left),
        _scale_throttle(command.right),
        flags,
    )


def unpack_drive_command(body: bytes) -> DriveCommand:
    """Unpack a drivetrain command body."""
    if len(body) != 5:
        raise ProtocolError(f"drive command body must be 5 bytes, got {len(body)}")
    left, right, flags = struct.unpack("<hhB", body)
    return DriveCommand(
        left=left / 10000.0,
        right=right / 10000.0,
        enabled=bool(flags & 0x01),
        estop_active=bool(flags & 0x02),
        motion_inhibited=bool(flags & 0x04),
    )


def pack_status(status: TeensyStatus) -> bytes:
    """Pack a Teensy status body."""
    flags = int(status.estop_active) | (int(status.motion_inhibited) << 1)
    online = int(status.controller_online[0]) | (int(status.controller_online[1]) << 1)
    return struct.pack("<BHBB", status.state, status.fault_code, flags, online)


def unpack_status(body: bytes) -> TeensyStatus:
    """Unpack a Teensy status body."""
    if len(body) != 5:
        raise ProtocolError(f"status body must be 5 bytes, got {len(body)}")
    state, fault_code, flags, online = struct.unpack("<BHBB", body)
    return TeensyStatus(
        state=state,
        fault_code=fault_code,
        estop_active=bool(flags & 0x01),
        motion_inhibited=bool(flags & 0x02),
        controller_online=(bool(online & 0x01), bool(online & 0x02)),
    )


def pack_drivetrain_telemetry(telemetry: DrivetrainTelemetry) -> bytes:
    """Pack drivetrain telemetry."""
    velocities_millirps = [
        int(round(value * 1000.0))
        for value in telemetry.wheel_velocity_rps
    ]
    flags = int(telemetry.estop_active) | (int(telemetry.motion_inhibited) << 1)
    online = int(telemetry.controller_online[0]) | (
        int(telemetry.controller_online[1]) << 1
    )
    return struct.pack(
        "<4i4hHBB",
        *telemetry.encoder_ticks,
        *velocities_millirps,
        telemetry.fault_code,
        flags,
        online,
    )


def unpack_drivetrain_telemetry(body: bytes) -> DrivetrainTelemetry:
    """Unpack drivetrain telemetry."""
    if len(body) != 28:
        raise ProtocolError(f"telemetry body must be 28 bytes, got {len(body)}")
    unpacked = struct.unpack("<4i4hHBB", body)
    ticks = tuple(int(value) for value in unpacked[:4])
    velocities = tuple(float(value) / 1000.0 for value in unpacked[4:8])
    fault_code = int(unpacked[8])
    flags = int(unpacked[9])
    online = int(unpacked[10])
    return DrivetrainTelemetry(
        encoder_ticks=ticks,
        wheel_velocity_rps=velocities,
        fault_code=fault_code,
        estop_active=bool(flags & 0x01),
        motion_inhibited=bool(flags & 0x02),
        controller_online=(bool(online & 0x01), bool(online & 0x02)),
    )
