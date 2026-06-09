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
ACTUATOR_1_DUTY = 230
ACTUATOR_2_DUTY = 255
DEPOSITION_ACTUATOR_DUTY = 255


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
    bldc_speed: int = 0
    bldc_pg_count: int = 0
    bldc_alarm_active: bool = False


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


def bldc_to_bytes(speed: int) -> bytes:
    clamped = max(-MAX_COMMAND, min(MAX_COMMAND, int(speed)))
    return f"B {clamped}\n".encode("ascii")


def send_bldc_cmd(port: serial.Serial, speed: int) -> None:
    port.write(bldc_to_bytes(speed))


def parse_telemetry_line(line: bytes | str) -> TeensyTelemetry | None:
    """Parse a Teensy telemetry line, returning ``None`` for other log lines."""
    text = line.decode("ascii", errors="replace") if isinstance(line, bytes) else line
    fields = text.strip().split()
    if not fields or fields[0] != "T":
        return None
    if len(fields) not in {13, 16}:
        raise ValueError(f"expected 13 or 16 telemetry fields, got {len(fields)}")

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
        bldc_speed=int(fields[13]) if len(fields) == 16 else 0,
        bldc_pg_count=int(fields[14]) if len(fields) == 16 else 0,
        bldc_alarm_active=bool(int(fields[15])) if len(fields) == 16 else False,
    )


def _clamp_actuator_duty(value: int) -> int:
    return max(0, min(255, int(value)))


def actuator_to_bytes(
    dir1: int,
    dir2: int,
    duty1: int = ACTUATOR_1_DUTY,
    duty2: int = ACTUATOR_2_DUTY,
    command: str = "C",
) -> bytes:
    d1 = max(-1, min(1, int(dir1)))
    d2 = max(-1, min(1, int(dir2)))
    p1 = _clamp_actuator_duty(duty1)
    p2 = _clamp_actuator_duty(duty2)
    command_name = command[:1].upper()
    if command_name not in {"C", "D"}:
        raise ValueError(f"unsupported actuator command: {command!r}")
    eol = chr(10)
    return (
        command_name
        + " "
        + str(d1)
        + " "
        + str(d2)
        + " "
        + str(p1)
        + " "
        + str(p2)
        + eol
    ).encode("ascii")


def send_actuator_cmd(
    port,
    dir1: int,
    dir2: int,
    duty1: int = ACTUATOR_1_DUTY,
    duty2: int = ACTUATOR_2_DUTY,
) -> None:
    port.write(actuator_to_bytes(dir1, dir2, duty1, duty2))


def deposition_actuator_to_bytes(
    dir1: int,
    dir2: int,
    duty1: int = DEPOSITION_ACTUATOR_DUTY,
    duty2: int = DEPOSITION_ACTUATOR_DUTY,
) -> bytes:
    return actuator_to_bytes(dir1, dir2, duty1, duty2, command="D")


def send_deposition_actuator_cmd(
    port,
    dir1: int,
    dir2: int,
    duty1: int = DEPOSITION_ACTUATOR_DUTY,
    duty2: int = DEPOSITION_ACTUATOR_DUTY,
) -> None:
    port.write(deposition_actuator_to_bytes(dir1, dir2, duty1, duty2))
