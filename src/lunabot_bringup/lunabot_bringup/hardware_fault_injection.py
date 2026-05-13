"""Publish deterministic hardware-boundary fault scenarios."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message

from lunabot_bringup.message_fields import write_message_field


@dataclass(frozen=True)
class TopicInjection:
    """One topic and message payload published by a fault scenario."""

    name: str
    type_name: str
    fields: dict[str, Any]
    rate_hz: float
    transient_local: bool = False


@dataclass(frozen=True)
class FaultScenario:
    """A named group of topic injections."""

    name: str
    description: str
    topics: tuple[TopicInjection, ...]


def _load_yaml(path: Path) -> dict[str, Any]:
    """Load and validate a YAML mapping."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def _message_from_config(type_name: str, fields: dict[str, Any]):
    """Build a ROS message from a type name and field mapping."""
    message_type = get_message(type_name)
    message = message_type()
    for path, value in fields.items():
        write_message_field(message, path, value)
    return message


def _topic_from_config(raw: Any) -> TopicInjection:
    """Parse one topic entry from the scenario config."""
    if not isinstance(raw, dict):
        raise ValueError("topic entries must be mappings")

    name = str(raw.get("name", "")).strip()
    type_name = str(raw.get("type", "")).strip()
    fields = raw.get("fields", {})
    rate_hz = float(raw.get("rate_hz", 5.0))
    transient_local = bool(raw.get("transient_local", False))

    if not name.startswith("/"):
        raise ValueError(f"topic name must be absolute: {name!r}")
    if not type_name:
        raise ValueError(f"{name} must define a message type")
    if not isinstance(fields, dict):
        raise ValueError(f"{name} fields must be a mapping")
    if rate_hz <= 0.0:
        raise ValueError(f"{name} rate_hz must be positive")

    return TopicInjection(
        name=name,
        type_name=type_name,
        fields=fields,
        rate_hz=rate_hz,
        transient_local=transient_local,
    )


def load_scenarios(path: Path) -> dict[str, FaultScenario]:
    """Load all fault scenarios from a YAML config."""
    data = _load_yaml(path)
    raw_scenarios = data.get("scenarios")
    if not isinstance(raw_scenarios, dict):
        raise ValueError("config must contain a scenarios mapping")

    scenarios: dict[str, FaultScenario] = {}
    for name, raw_scenario in raw_scenarios.items():
        if not isinstance(raw_scenario, dict):
            raise ValueError(f"scenario {name!r} must be a mapping")
        raw_topics = raw_scenario.get("topics", [])
        if not isinstance(raw_topics, list) or not raw_topics:
            raise ValueError(f"scenario {name!r} must define topic entries")
        scenarios[str(name)] = FaultScenario(
            name=str(name),
            description=str(raw_scenario.get("description", "")),
            topics=tuple(_topic_from_config(item) for item in raw_topics),
        )
    return scenarios


def scenario_needs_live_topic_ack(scenario: FaultScenario) -> bool:
    """Return whether a scenario publishes high-impact live stack topics."""
    return any(topic.name.startswith("/safety/") for topic in scenario.topics)


def validate_live_topic_ack(
    scenario: FaultScenario,
    *,
    allow_live_topics: bool,
) -> None:
    """Reject live safety-topic publishing without an explicit operator ack."""
    if scenario_needs_live_topic_ack(scenario) and not allow_live_topics:
        raise ValueError(
            f"scenario '{scenario.name}' publishes /safety topics; rerun with "
            "--allow-live-topics only in an isolated test ROS_DOMAIN_ID"
        )


class HardwareFaultInjector(Node):
    """Publish one hardware fault scenario on real project ROS interfaces."""

    def __init__(self, scenario: FaultScenario) -> None:
        """Create publishers and periodic timers for a scenario."""
        super().__init__("hardware_fault_injector")
        self._scenario = scenario
        for topic in scenario.topics:
            self._add_topic(topic)
        self.get_logger().info(
            f"Hardware fault scenario '{scenario.name}' publishing "
            f"{len(scenario.topics)} topic(s)"
        )

    def _add_topic(self, topic: TopicInjection) -> None:
        """Create one publisher/timer pair for a scenario topic."""
        message_type = get_message(topic.type_name)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if topic.transient_local
                else DurabilityPolicy.VOLATILE
            ),
        )
        publisher = self.create_publisher(message_type, topic.name, qos)

        def _publish() -> None:
            message = _message_from_config(topic.type_name, topic.fields)
            if hasattr(message, "header"):
                message.header.stamp = self.get_clock().now().to_msg()
            if hasattr(message, "stamp"):
                message.stamp = self.get_clock().now().to_msg()
            publisher.publish(message)

        self.create_timer(1.0 / topic.rate_hz, _publish)
        _publish()


def _default_config_path() -> Path:
    """Return the installed default hardware fault scenario config."""
    share_dir = Path(get_package_share_directory("lunabot_bringup"))
    return share_dir / "config" / "hardware_fault_scenarios.yaml"


def _arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        description="Publish hardware-boundary mock faults on ROS interfaces"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Fault scenario YAML; defaults to the installed bringup config",
    )
    parser.add_argument(
        "--scenario",
        required=True,
        help="Scenario name from the YAML config",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Publish for this long, or run until interrupted when <= 0",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available scenarios and exit",
    )
    parser.add_argument(
        "--allow-live-topics",
        action="store_true",
        help="Allow scenarios that publish live safety topics such as /safety/estop",
    )
    return parser


def main(args=None) -> None:
    """Run the hardware fault injector."""
    parser = _arg_parser()
    parsed = parser.parse_args(args)
    config_path = parsed.config or _default_config_path()
    scenarios = load_scenarios(config_path)

    if parsed.list:
        for scenario in scenarios.values():
            print(f"{scenario.name}: {scenario.description}")
        return

    scenario = scenarios.get(parsed.scenario)
    if scenario is None:
        parser.error(
            f"unknown scenario {parsed.scenario!r}; "
            f"available: {', '.join(sorted(scenarios))}"
        )
    assert scenario is not None
    try:
        validate_live_topic_ack(
            scenario,
            allow_live_topics=bool(parsed.allow_live_topics),
        )
    except ValueError as exc:
        parser.error(str(exc))

    rclpy.init(args=None)
    node = HardwareFaultInjector(scenario)
    try:
        if parsed.duration_s > 0.0:
            deadline = time.monotonic() + parsed.duration_s
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
