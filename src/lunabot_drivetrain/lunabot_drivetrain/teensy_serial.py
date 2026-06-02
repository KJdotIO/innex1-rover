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

"""USB serial line protocol helpers for the Teensy drivetrain firmware."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import serial


MAX_COMMAND = 127


@dataclass(frozen=True)
class TeensyTelemetry:
    """One parsed ``T`` telemetry line from the Teensy firmware."""

    millis: int
    state: str
    estop_active: bool
    motion_inhibited: bool
    target_left: int
    target_right: int
    command_left: int
    command_right: int
    encoder_ticks: list[int]


def throttle_to_command(value: float) -> int:
    """Convert a normalised ROS throttle to a signed Teensy command."""
    clamped = max(-1.0, min(1.0, value))
    return int(round(clamped * MAX_COMMAND))


def throttle_to_bytes(left: float, right: float) -> bytes:
    """Build a Teensy ``V left right`` velocity command."""
    return f"V {throttle_to_command(left)} {throttle_to_command(right)}\n".encode(
        "ascii"
    )


def stop_to_bytes() -> bytes:
    """Build a Teensy stop command that holds the software latch clear."""
    return b"X\n"


def estop_to_bytes() -> bytes:
    """Build a Teensy software e-stop latch command."""
    return b"E\n"


def release_estop_to_bytes() -> bytes:
    """Build a Teensy software e-stop release command."""
    return b"U\n"


def restart_to_bytes() -> bytes:
    """Build a Teensy restart command after a software e-stop."""
    return b"R\n"


def zero_encoders_to_bytes() -> bytes:
    """Build a Teensy encoder zeroing command."""
    return b"Z\n"


def send_throttle(port: serial.Serial, left: float, right: float) -> None:
    """Send per-side drivetrain throttle to the Teensy."""
    port.write(throttle_to_bytes(left, right))


def send_stop(port: serial.Serial) -> None:
    """Send a hard stop to the Teensy."""
    port.write(stop_to_bytes())


def send_estop(port: serial.Serial) -> None:
    """Latch the Teensy firmware into software e-stop."""
    port.write(estop_to_bytes())


def send_release_estop(port: serial.Serial) -> None:
    """Release the Teensy software e-stop latch."""
    port.write(release_estop_to_bytes())


def send_restart(port: serial.Serial) -> None:
    """Restart motion after the Teensy software e-stop latch is released."""
    port.write(restart_to_bytes())


def send_zero_encoders(port: serial.Serial) -> None:
    """Reset encoder counts in the Teensy firmware."""
    port.write(zero_encoders_to_bytes())


def parse_telemetry_line(line: bytes | str) -> TeensyTelemetry | None:
    """Parse a Teensy telemetry line, returning ``None`` for other log lines."""
    text = line.decode("ascii", errors="replace") if isinstance(line, bytes) else line
    fields = text.strip().split()
    if not fields or fields[0] != "T":
        return None
    if len(fields) != 13:
        raise ValueError(f"expected 13 telemetry fields, got {len(fields)}")

    return TeensyTelemetry(
        millis=int(fields[1]),
        state=fields[2],
        estop_active=bool(int(fields[3])),
        motion_inhibited=bool(int(fields[4])),
        target_left=int(fields[5]),
        target_right=int(fields[6]),
        command_left=int(fields[7]),
        command_right=int(fields[8]),
        # Firmware prints encoder counts as FL RL FR RR. ROS messages in this
        # package use FL FR RL RR, matching ``_WHEEL_NAMES``.
        encoder_ticks=[
            int(fields[9]),
            int(fields[11]),
            int(fields[10]),
            int(fields[12]),
        ],
    )
