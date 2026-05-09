# pyright: reportAttributeAccessIssue=false
"""Publish and classify rover power telemetry."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from lunabot_interfaces.msg import PowerTelemetry


@dataclass(frozen=True)
class PowerProfile:
    """Voltage thresholds for one battery profile."""

    name: str
    cells: int
    warning_voltage_v: float
    critical_voltage_v: float


def load_power_profiles(path: Path) -> dict[str, PowerProfile]:
    """Load named power profiles from a YAML file."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw_profiles = data.get("profiles") if isinstance(data, dict) else None
    if not isinstance(raw_profiles, dict):
        raise ValueError(f"{path} must contain a profiles mapping")

    profiles: dict[str, PowerProfile] = {}
    for name, raw_profile in raw_profiles.items():
        if not isinstance(raw_profile, dict):
            raise ValueError(f"profile {name!r} must be a mapping")
        profile = PowerProfile(
            name=str(name),
            cells=int(raw_profile["cells"]),
            warning_voltage_v=float(raw_profile["warning_voltage_v"]),
            critical_voltage_v=float(raw_profile["critical_voltage_v"]),
        )
        if profile.cells <= 0:
            raise ValueError(f"profile {name!r} cells must be positive")
        if profile.critical_voltage_v >= profile.warning_voltage_v:
            raise ValueError(
                f"profile {name!r} critical voltage must be below warning voltage"
            )
        profiles[profile.name] = profile
    return profiles


def classify_power_state(
    voltage_v: float | None,
    profile: PowerProfile,
) -> int:
    """Return the power state for one voltage sample."""
    if voltage_v is None or voltage_v <= 0.0:
        return PowerTelemetry.STATE_UNAVAILABLE
    if voltage_v <= profile.critical_voltage_v:
        return PowerTelemetry.STATE_LOW_CRITICAL
    if voltage_v <= profile.warning_voltage_v:
        return PowerTelemetry.STATE_LOW_WARNING
    return PowerTelemetry.STATE_OK


def build_power_telemetry(
    *,
    voltage_v: float | None,
    current_a: float,
    energy_wh: float,
    profile: PowerProfile,
    source: str,
) -> PowerTelemetry:
    """Build one power telemetry message from the current source values."""
    state = classify_power_state(voltage_v, profile)
    msg = PowerTelemetry()
    msg.profile = profile.name
    msg.source = source
    msg.state = state
    msg.bus_voltage_v = float(voltage_v) if voltage_v is not None else 0.0
    msg.bus_current_a = float(current_a)
    msg.energy_wh = float(energy_wh)
    msg.warning_voltage_v = profile.warning_voltage_v
    msg.critical_voltage_v = profile.critical_voltage_v
    msg.low_voltage_warning = state in {
        PowerTelemetry.STATE_LOW_WARNING,
        PowerTelemetry.STATE_LOW_CRITICAL,
    }
    msg.low_voltage_critical = state == PowerTelemetry.STATE_LOW_CRITICAL
    return msg


def _package_config_path(filename: str) -> Path:
    share = get_package_share_directory("lunabot_bringup")
    return Path(share) / "config" / filename


class ManualPowerTelemetryNode(Node):
    """Publish operator-entered power telemetry until a hardware source exists."""

    def __init__(self) -> None:
        super().__init__("manual_power_telemetry")
        self.declare_parameter(
            "profiles_path",
            str(_package_config_path("power_profiles.yaml")),
        )
        self.declare_parameter("profile", "lipo_6s")
        self.declare_parameter("source", "manual")
        self.declare_parameter("bus_voltage_v", -1.0)
        self.declare_parameter("bus_current_a", 0.0)
        self.declare_parameter("energy_wh", 0.0)
        self.declare_parameter("publish_hz", 1.0)

        profiles_path = Path(str(self.get_parameter("profiles_path").value))
        self._profiles = load_power_profiles(profiles_path)
        self._profile_name = str(self.get_parameter("profile").value)
        if self._profile_name not in self._profiles:
            supported = ", ".join(sorted(self._profiles))
            raise ValueError(
                f"Unknown power profile {self._profile_name!r}; "
                f"expected one of: {supported}"
            )

        publish_hz = float(self.get_parameter("publish_hz").value)
        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")

        self._publisher = self.create_publisher(
            PowerTelemetry, "/power/telemetry", 10
        )
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            "Manual power telemetry publishing on /power/telemetry"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _current_message(self) -> PowerTelemetry:
        voltage = self._float_parameter("bus_voltage_v")
        msg = build_power_telemetry(
            voltage_v=voltage if voltage > 0.0 else None,
            current_a=self._float_parameter("bus_current_a"),
            energy_wh=self._float_parameter("energy_wh"),
            profile=self._profiles[self._profile_name],
            source=self._string_parameter("source"),
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _publish(self) -> None:
        self._publisher.publish(self._current_message())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualPowerTelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
