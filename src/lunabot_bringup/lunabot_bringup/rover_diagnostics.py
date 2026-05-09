"""Publish standard ROS diagnostics for rover runtime readiness."""

from __future__ import annotations

import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Bool

from lunabot_interfaces import msg as lunabot_msgs

LEVEL_OK = DiagnosticStatus.OK
LEVEL_WARN = DiagnosticStatus.WARN
LEVEL_ERROR = DiagnosticStatus.ERROR
LEVEL_STALE = DiagnosticStatus.STALE

DRIVETRAIN_STATE_FAULT = 4
DRIVETRAIN_STATE_ESTOP = 5
EXCAVATION_STATE_FAULT = 6
EXCAVATION_FAULT_NONE = 0

DrivetrainStatus: Any = lunabot_msgs.__dict__["DrivetrainStatus"]
ExcavationStatus: Any = lunabot_msgs.__dict__["ExcavationStatus"]
LocalisationStartZoneStatus: Any = lunabot_msgs.__dict__[
    "LocalisationStartZoneStatus"
]
PowerTelemetry: Any = lunabot_msgs.__dict__["PowerTelemetry"]


@dataclass
class TopicSample:
    """Latest message and wall-clock receipt time for one monitored topic."""

    msg: Any | None = None
    received_at_s: float | None = None


def _key_value(key: str, value: object) -> KeyValue:
    item = KeyValue()
    item.key = key
    item.value = str(value)
    return item


def _status(
    name: str,
    level: int,
    message: str,
    *,
    hardware_id: str = "innex1_rover",
    values: Mapping[str, object] | None = None,
) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.name = name
    status.hardware_id = hardware_id
    status.level = level
    status.message = message
    status.values = [
        _key_value(key, value) for key, value in (values or {}).items()
    ]
    return status


def _sample_age_s(sample: TopicSample, now_s: float) -> float | None:
    if sample.received_at_s is None:
        return None
    return max(0.0, now_s - sample.received_at_s)


def _is_stale(sample: TopicSample, now_s: float, timeout_s: float) -> bool:
    age_s = _sample_age_s(sample, now_s)
    return age_s is None or age_s > timeout_s


def drivetrain_diagnostic(
    sample: TopicSample,
    *,
    now_s: float,
    timeout_s: float,
) -> DiagnosticStatus:
    """Build drivetrain health from the latest status message."""
    age_s = _sample_age_s(sample, now_s)
    if _is_stale(sample, now_s, timeout_s):
        return _status(
            "drivetrain/status",
            LEVEL_STALE,
            "No fresh drivetrain status",
            values={"age_s": age_s if age_s is not None else "never"},
        )

    msg = sample.msg
    if msg is None:
        return _status(
            "drivetrain/status",
            LEVEL_STALE,
            "No fresh drivetrain status",
            values={"age_s": age_s if age_s is not None else "never"},
        )

    values = {
        "state": msg.state,
        "fault_code": msg.fault_code,
        "controller_online": list(msg.controller_online),
        "age_s": f"{age_s:.2f}",
    }
    if msg.state in {
        DRIVETRAIN_STATE_FAULT,
        DRIVETRAIN_STATE_ESTOP,
    } or msg.estop_active:
        return _status(
            "drivetrain/status",
            LEVEL_ERROR,
            "Drivetrain fault or E-stop active",
            values=values,
        )
    if msg.motion_inhibited:
        return _status(
            "drivetrain/status",
            LEVEL_WARN,
            "Motion inhibited",
            values=values,
        )
    if not all(msg.controller_online):
        return _status(
            "drivetrain/status",
            LEVEL_WARN,
            "One or more drivetrain controllers offline",
            values=values,
        )
    return _status(
        "drivetrain/status",
        LEVEL_OK,
        "Drivetrain ready",
        values=values,
    )


def safety_diagnostic(
    estop: TopicSample,
    inhibit: TopicSample,
    *,
    now_s: float,
    timeout_s: float,
) -> DiagnosticStatus:
    """Build safety health from E-stop and motion-inhibit topics."""
    estop_age = _sample_age_s(estop, now_s)
    inhibit_age = _sample_age_s(inhibit, now_s)
    values = {
        "estop_age_s": estop_age if estop_age is not None else "never",
        "motion_inhibit_age_s": (
            inhibit_age if inhibit_age is not None else "never"
        ),
    }
    if _is_stale(estop, now_s, timeout_s) or _is_stale(
        inhibit, now_s, timeout_s
    ):
        return _status(
            "safety/command_state",
            LEVEL_STALE,
            "No fresh safety state",
            values=values,
        )

    estop_msg = estop.msg
    inhibit_msg = inhibit.msg
    if estop_msg is None or inhibit_msg is None:
        return _status(
            "safety/command_state",
            LEVEL_STALE,
            "No fresh safety state",
            values=values,
        )

    estop_active = bool(estop_msg.data)
    motion_inhibited = bool(inhibit_msg.data)
    values.update(
        {
            "estop_active": estop_active,
            "motion_inhibited": motion_inhibited,
        }
    )
    if estop_active:
        return _status(
            "safety/command_state",
            LEVEL_ERROR,
            "E-stop active",
            values=values,
        )
    if motion_inhibited:
        return _status(
            "safety/command_state",
            LEVEL_WARN,
            "Motion inhibited",
            values=values,
        )
    return _status(
        "safety/command_state",
        LEVEL_OK,
        "Safety state clear",
        values=values,
    )


def localisation_diagnostic(
    sample: TopicSample,
    *,
    now_s: float,
    timeout_s: float,
) -> DiagnosticStatus:
    """Build start-zone localisation health."""
    age_s = _sample_age_s(sample, now_s)
    if _is_stale(sample, now_s, timeout_s):
        return _status(
            "localisation/start_zone",
            LEVEL_STALE,
            "No fresh start-zone localisation status",
            values={"age_s": age_s if age_s is not None else "never"},
        )

    msg = sample.msg
    if msg is None:
        return _status(
            "localisation/start_zone",
            LEVEL_STALE,
            "No fresh start-zone localisation status",
            values={"age_s": age_s if age_s is not None else "never"},
        )

    values = {
        "state": msg.state,
        "ready": msg.ready,
        "reason_code": msg.reason_code,
        "stable_lock_age_s": f"{msg.stable_lock_age_s:.2f}",
        "age_s": f"{age_s:.2f}",
    }
    if msg.ready:
        return _status(
            "localisation/start_zone",
            LEVEL_OK,
            "Start-zone localisation ready",
            values=values,
        )
    return _status(
        "localisation/start_zone",
        LEVEL_WARN,
        msg.reason_detail or "Start-zone localisation not ready",
        values=values,
    )


def excavation_diagnostic(
    sample: TopicSample,
    *,
    now_s: float,
    timeout_s: float,
) -> DiagnosticStatus:
    """Build excavation subsystem health."""
    age_s = _sample_age_s(sample, now_s)
    if _is_stale(sample, now_s, timeout_s):
        return _status(
            "excavation/status",
            LEVEL_STALE,
            "No fresh excavation status",
            values={"age_s": age_s if age_s is not None else "never"},
        )

    msg = sample.msg
    if msg is None:
        return _status(
            "excavation/status",
            LEVEL_STALE,
            "No fresh excavation status",
            values={"age_s": age_s if age_s is not None else "never"},
        )

    values = {
        "state": msg.state,
        "fault_code": msg.fault_code,
        "estop_active": msg.estop_active,
        "driver_fault": msg.driver_fault,
        "homed": msg.homed,
        "motor_enabled": msg.motor_enabled,
        "motor_current_a": f"{msg.motor_current_a:.2f}",
        "age_s": f"{age_s:.2f}",
    }
    if (
        msg.state == EXCAVATION_STATE_FAULT
        or msg.fault_code != EXCAVATION_FAULT_NONE
        or msg.estop_active
        or msg.driver_fault
    ):
        return _status(
            "excavation/status",
            LEVEL_ERROR,
            "Excavation fault active",
            values=values,
        )
    if not msg.homed:
        return _status(
            "excavation/status",
            LEVEL_WARN,
            "Excavation not homed",
            values=values,
        )
    return _status(
        "excavation/status",
        LEVEL_OK,
        "Excavation ready",
        values=values,
    )


def power_diagnostic(
    sample: TopicSample,
    *,
    now_s: float,
    timeout_s: float,
) -> DiagnosticStatus:
    """Build power health from the latest power telemetry sample."""
    age_s = _sample_age_s(sample, now_s)
    if _is_stale(sample, now_s, timeout_s):
        return _status(
            "power/telemetry",
            LEVEL_STALE,
            "No fresh power telemetry",
            values={"age_s": age_s if age_s is not None else "never"},
        )

    msg = sample.msg
    if msg is None:
        return _status(
            "power/telemetry",
            LEVEL_STALE,
            "No fresh power telemetry",
            values={"age_s": age_s if age_s is not None else "never"},
        )

    values = {
        "profile": msg.profile,
        "source": msg.source,
        "state": msg.state,
        "bus_voltage_v": f"{msg.bus_voltage_v:.2f}",
        "bus_current_a": f"{msg.bus_current_a:.2f}",
        "energy_wh": f"{msg.energy_wh:.2f}",
        "warning_voltage_v": f"{msg.warning_voltage_v:.2f}",
        "critical_voltage_v": f"{msg.critical_voltage_v:.2f}",
        "age_s": f"{age_s:.2f}",
    }
    if msg.state == PowerTelemetry.STATE_UNAVAILABLE:
        return _status(
            "power/telemetry",
            LEVEL_STALE,
            "Power telemetry unavailable",
            values=values,
        )
    if msg.low_voltage_critical:
        return _status(
            "power/telemetry",
            LEVEL_ERROR,
            "Power voltage critical",
            values=values,
        )
    if msg.low_voltage_warning:
        return _status(
            "power/telemetry",
            LEVEL_WARN,
            "Power voltage low",
            values=values,
        )
    return _status(
        "power/telemetry",
        LEVEL_OK,
        "Power telemetry OK",
        values=values,
    )


class RoverDiagnostics(Node):
    """Aggregate rover health into `/diagnostics`."""

    def __init__(self) -> None:
        super().__init__("rover_diagnostics")
        self.declare_parameter("stale_timeout_s", 2.5)
        self.declare_parameter("publish_hz", 1.0)

        self._stale_timeout_s = float(
            self.get_parameter("stale_timeout_s").value
        )
        publish_hz = float(self.get_parameter("publish_hz").value)
        if self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive")
        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")

        self._drivetrain = TopicSample()
        self._estop = TopicSample()
        self._motion_inhibit = TopicSample()
        self._localisation = TopicSample()
        self._excavation = TopicSample()
        self._power = TopicSample()

        self.create_subscription(
            DrivetrainStatus,
            "/drivetrain/status",
            lambda msg: self._store(self._drivetrain, msg),
            10,
        )
        self.create_subscription(
            Bool,
            "/safety/estop",
            lambda msg: self._store(self._estop, msg),
            10,
        )
        self.create_subscription(
            Bool,
            "/safety/motion_inhibit",
            lambda msg: self._store(self._motion_inhibit, msg),
            10,
        )
        self.create_subscription(
            LocalisationStartZoneStatus,
            "/localisation/start_zone_status",
            lambda msg: self._store(self._localisation, msg),
            10,
        )
        self.create_subscription(
            ExcavationStatus,
            "/excavation/status",
            lambda msg: self._store(self._excavation, msg),
            10,
        )
        self.create_subscription(
            PowerTelemetry,
            "/power/telemetry",
            lambda msg: self._store(self._power, msg),
            10,
        )
        self._publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info("Rover diagnostics publishing on /diagnostics")

    def _store(self, sample: TopicSample, msg: Any) -> None:
        sample.msg = msg
        sample.received_at_s = time.monotonic()

    def _publish(self) -> None:
        now_s = time.monotonic()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [
            safety_diagnostic(
                self._estop,
                self._motion_inhibit,
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            ),
            drivetrain_diagnostic(
                self._drivetrain,
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            ),
            localisation_diagnostic(
                self._localisation,
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            ),
            excavation_diagnostic(
                self._excavation,
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            ),
            power_diagnostic(
                self._power,
                now_s=now_s,
                timeout_s=self._stale_timeout_s,
            ),
        ]
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
