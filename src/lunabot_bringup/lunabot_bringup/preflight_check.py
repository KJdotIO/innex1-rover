"""Run startup preflight checks for simulation bringup."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from lunabot_interfaces.action import Deposit
from lunabot_interfaces.action import Excavate
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from tf2_ros import TransformListener
import yaml

from lunabot_bringup.preflight_profiles import filter_preflight_config
from lunabot_bringup.preflight_profiles import PHASE_FULL
from lunabot_bringup.preflight_profiles import VALID_PHASES


EXIT_OK = 0
EXIT_CRITICAL_FAILURE = 2
EXIT_INTERNAL_ERROR = 3

ACTION_TYPE_MAP: dict[str, type] = {
    "lunabot_interfaces/action/Deposit": Deposit,
    "lunabot_interfaces/action/Excavate": Excavate,
    "nav2_msgs/action/NavigateToPose": NavigateToPose,
}

TRUE_STRINGS = {"1", "true", "yes", "on"}
FALSE_STRINGS = {"0", "false", "no", "off"}


@dataclass
class CheckResult:
    """Result item for one preflight check."""

    name: str
    status: str
    critical: bool
    detail: str
    elapsed_s: float


def _load_yaml(path: Path) -> dict[str, Any]:
    """Load preflight YAML config."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Invalid preflight config in {path}")
    return data


def _repo_contract_path() -> Path:
    """Locate interface contract file when run from repo root."""
    return Path.cwd() / ".github" / "contracts" / "interface_contracts.json"


def _merge_contract_requirements(config: dict[str, Any], logger) -> dict[str, Any]:
    """Merge contract-defined topics and TF checks into preflight config."""
    preflight = config.setdefault("preflight", {})
    use_contracts = bool(preflight.get("use_interface_contracts", True))
    if not use_contracts:
        logger.info("Interface contract merge disabled by config")
        return config

    contract_path = _repo_contract_path()
    if not contract_path.exists():
        logger.info("No local interface contract file found")
        return config

    try:
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        logger.warning(f"Could not parse {contract_path}: {exc}")
        return config

    topics = preflight.setdefault("required_topics", [])
    topic_keys = {(item.get("name"), item.get("type")) for item in topics}
    for item in contract.get("topics", []):
        if not isinstance(item, dict):
            continue
        if item.get("kind") != "publisher":
            continue
        key = (item.get("name"), item.get("type"))
        if key in topic_keys:
            continue
        topics.append(
            {
                "name": item.get("name"),
                "type": item.get("type"),
                "min_messages": 1,
                "critical": True,
                "phases": item.get("phases"),
            }
        )
        logger.info(f"Added contract topic check: {item.get('name')}")
        topic_keys.add(key)

    tf_links = preflight.setdefault("required_tf_links", [])
    tf_keys = {(item.get("parent"), item.get("child")) for item in tf_links}
    for item in contract.get("tf_links", []):
        if not isinstance(item, dict):
            continue
        key = (item.get("parent"), item.get("child"))
        if key in tf_keys:
            continue
        tf_links.append(
            {
                "parent": item.get("parent"),
                "child": item.get("child"),
                "critical": True,
                "phases": item.get("phases"),
            }
        )
        parent = item.get("parent")
        child = item.get("child")
        logger.info(f"Added contract TF check: {parent}->{child}")
        tf_keys.add(key)

    return config


class PreflightChecker(Node):
    """Execute preflight checks before autonomy startup."""

    def __init__(
        self,
        config: dict[str, Any],
        phase: str = PHASE_FULL,
        use_sim_time: bool = True,
    ):
        """Create checker node with merged runtime config."""
        super().__init__(
            "preflight_check",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, bool(use_sim_time))
            ],
        )
        merged_config = _merge_contract_requirements(config, self.get_logger())
        self._config = filter_preflight_config(merged_config, phase)
        self._results: list[CheckResult] = []

    def _record(
        self,
        name: str,
        critical: bool,
        ok: bool,
        detail: str,
        started: float,
    ) -> None:
        """Store and print one check result."""
        status = "PASS" if ok else "FAIL"
        elapsed = time.monotonic() - started
        self._results.append(
            CheckResult(
                name=name,
                status=status,
                critical=critical,
                detail=detail,
                elapsed_s=elapsed,
            )
        )
        print(f"[{status}] {name}: {detail} ({elapsed:.2f}s)")

    def _check_sim_time(self) -> None:
        """Check that simulated clock advances."""
        preflight = self._config["preflight"]
        sim_cfg = preflight.get("sim_time", {})
        if not bool(self.get_parameter("use_sim_time").value):
            return
        if not bool(sim_cfg.get("enabled", True)):
            return
        timeout_s = float(sim_cfg.get("timeout_s", 5.0))
        min_delta_s = float(sim_cfg.get("min_delta_s", 0.2))
        critical = bool(sim_cfg.get("critical", True))

        started = time.monotonic()
        start_ns = self.get_clock().now().nanoseconds
        deadline = started + timeout_s

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - start_ns) >= int(min_delta_s * 1e9):
                delta = (now_ns - start_ns) / 1e9
                self._record(
                    "sim_time",
                    critical,
                    True,
                    f"/clock advanced by {delta:.3f}s",
                    started,
                )
                return

        self._record(
            "sim_time",
            critical,
            False,
            f"/clock did not advance by at least {min_delta_s}s",
            started,
        )

    def _topic_types(self) -> dict[str, list[str]]:
        """Return discovered topic types keyed by topic name."""
        return {name: types for name, types in self.get_topic_names_and_types()}

    @staticmethod
    def _parse_reliability(value: str) -> QoSReliabilityPolicy:
        """Return QoS reliability from config text."""
        mapping = {
            "best_effort": QoSReliabilityPolicy.BEST_EFFORT,
            "reliable": QoSReliabilityPolicy.RELIABLE,
        }
        return mapping.get(value.lower(), QoSReliabilityPolicy.BEST_EFFORT)

    @staticmethod
    def _parse_durability(value: str) -> QoSDurabilityPolicy:
        """Return QoS durability from config text."""
        mapping = {
            "volatile": QoSDurabilityPolicy.VOLATILE,
            "transient_local": QoSDurabilityPolicy.TRANSIENT_LOCAL,
        }
        return mapping.get(value.lower(), QoSDurabilityPolicy.VOLATILE)

    def _check_required_topics(self) -> None:
        """Check required topics for type and message flow."""
        preflight = self._config["preflight"]
        required = preflight.get("required_topics", [])
        msg_timeout_s = float(preflight.get("topic_message_timeout_s", 5.0))
        discovery_timeout_s = float(
            preflight.get("topic_discovery_timeout_s", msg_timeout_s)
        )

        for topic_cfg in required:
            topic_name = topic_cfg["name"]
            expected_type = topic_cfg["type"]
            min_messages = int(topic_cfg.get("min_messages", 1))
            critical = bool(topic_cfg.get("critical", True))
            reliability = self._parse_reliability(
                str(topic_cfg.get("reliability", "best_effort"))
            )
            durability = self._parse_durability(
                str(topic_cfg.get("durability", "volatile"))
            )
            started = time.monotonic()

            discovery_deadline = time.monotonic() + discovery_timeout_s
            discovered_types: list[str] | None = None
            while time.monotonic() < discovery_deadline:
                discovered_types = self._topic_types().get(topic_name)
                if discovered_types:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

            if not discovered_types:
                self._record(
                    f"topic:{topic_name}",
                    critical,
                    False,
                    "topic not discovered within timeout",
                    started,
                )
                continue

            if expected_type not in discovered_types:
                detail = f"type mismatch expected={expected_type} "
                detail += f"got={discovered_types}"
                self._record(
                    f"topic:{topic_name}",
                    critical,
                    False,
                    detail,
                    started,
                )
                continue

            message_type = get_message(expected_type)
            received = {"count": 0}
            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=reliability,
                durability=durability,
            )

            subscription = self.create_subscription(
                message_type,
                topic_name,
                lambda _msg, bucket=received: bucket.__setitem__(
                    "count", bucket["count"] + 1
                ),
                qos,
            )

            deadline = time.monotonic() + msg_timeout_s
            while time.monotonic() < deadline and received["count"] < min_messages:
                rclpy.spin_once(self, timeout_sec=0.1)

            self.destroy_subscription(subscription)

            if received["count"] >= min_messages:
                self._record(
                    f"topic:{topic_name}",
                    critical,
                    True,
                    f"received {received['count']} msg(s)",
                    started,
                )
                continue

            detail = f"received {received['count']} msg(s), "
            detail += f"expected >= {min_messages}"
            self._record(
                f"topic:{topic_name}",
                critical,
                False,
                detail,
                started,
            )

    def _check_tf_links(self) -> None:
        """Check required TF transforms are available."""
        preflight = self._config["preflight"]
        required = preflight.get("required_tf_links", [])
        tf_timeout_s = float(preflight.get("tf_timeout_s", 5.0))

        buffer = Buffer()
        listener = TransformListener(buffer, self)

        try:
            for tf_cfg in required:
                parent = tf_cfg["parent"]
                child = tf_cfg["child"]
                critical = bool(tf_cfg.get("critical", True))
                started = time.monotonic()
                deadline = started + tf_timeout_s

                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if buffer.can_transform(parent, child, Time()):
                        self._record(
                            f"tf:{parent}->{child}",
                            critical,
                            True,
                            "transform available",
                            started,
                        )
                        break
                else:
                    self._record(
                        f"tf:{parent}->{child}",
                        critical,
                        False,
                        "transform unavailable within timeout",
                        started,
                    )
        finally:
            del listener

    def _check_actions(self) -> None:
        """Check required action servers are available."""
        preflight = self._config["preflight"]
        required = preflight.get("required_actions", [])
        timeout_s = float(preflight.get("action_timeout_s", 5.0))

        for action_cfg in required:
            action_name = action_cfg["name"]
            action_type_name = action_cfg["type"]
            critical = bool(action_cfg.get("critical", True))
            started = time.monotonic()

            action_type = ACTION_TYPE_MAP.get(action_type_name)
            if action_type is None:
                self._record(
                    f"action:{action_name}",
                    critical,
                    False,
                    f"unsupported action type mapping: {action_type_name}",
                    started,
                )
                continue

            client = ActionClient(self, action_type, action_name)
            available = client.wait_for_server(timeout_sec=timeout_s)
            client.destroy()

            if available:
                self._record(
                    f"action:{action_name}",
                    critical,
                    True,
                    "server available",
                    started,
                )
                continue

            self._record(
                f"action:{action_name}",
                critical,
                False,
                "server unavailable within timeout",
                started,
            )

    def _check_nodes(self) -> None:
        """Check required core nodes are visible on the graph."""
        preflight = self._config["preflight"]
        required = preflight.get("required_nodes", [])
        timeout_s = float(preflight.get("node_timeout_s", 5.0))

        for node_cfg in required:
            expected = node_cfg["name"]
            critical = bool(node_cfg.get("critical", True))
            started = time.monotonic()
            deadline = started + timeout_s

            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                discovered_nodes = self.get_node_names_and_namespaces()
                names = [name for name, _namespace in discovered_nodes]
                if expected in names:
                    self._record(
                        f"node:{expected}",
                        critical,
                        True,
                        "node discovered",
                        started,
                    )
                    break
            else:
                self._record(
                    f"node:{expected}",
                    critical,
                    False,
                    "node not discovered within timeout",
                    started,
                )

    def run(self) -> list[CheckResult]:
        """
        Run preflight checks.

        Execute all configured preflight checks in order.
        """
        self._check_sim_time()
        self._check_required_topics()
        self._check_tf_links()
        self._check_actions()
        self._check_nodes()
        return self._results


def _print_summary(results: list[CheckResult]) -> None:
    """Print human-readable summary."""
    passed = sum(1 for item in results if item.status == "PASS")
    failed = [item for item in results if item.status == "FAIL"]
    critical_failed = [item for item in failed if item.critical]

    print("\n=== Preflight Summary ===")
    print(f"Passed: {passed}")
    print(f"Failed: {len(failed)}")
    print(f"Critical failures: {len(critical_failed)}")
    if not failed:
        return

    print("Failed checks:")
    for item in failed:
        severity = "CRITICAL" if item.critical else "NON-CRITICAL"
        print(f"- [{severity}] {item.name}: {item.detail}")


def _write_json(path: Path, results: list[CheckResult]) -> None:
    """Write structured JSON summary."""
    payload = {
        "generated_at_unix": time.time(),
        "checks": [asdict(item) for item in results],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _exit_code(results: list[CheckResult]) -> int:
    """Return process exit code."""
    for item in results:
        if item.status == "FAIL" and item.critical:
            return EXIT_CRITICAL_FAILURE
    return EXIT_OK


def _arg_parser() -> argparse.ArgumentParser:
    """Create CLI argument parser."""
    parser = argparse.ArgumentParser(description="Run bringup preflight checks")
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to preflight config YAML",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Write JSON report to this file",
    )
    parser.add_argument(
        "--phase",
        default=PHASE_FULL,
        choices=sorted(VALID_PHASES),
        help=(
            "Preflight phase to run. Use 'launch' for the bringup gate or "
            "'full' for the full stack check."
        ),
    )
    parser.add_argument(
        "--use-sim-time",
        type=_parse_bool_text,
        default=True,
        metavar="{true,false}",
        help="Whether the preflight checker should require a live /clock.",
    )
    return parser


def _default_config_path() -> Path:
    """Resolve installed preflight config path."""
    share_dir = Path(get_package_share_directory("lunabot_bringup"))
    return share_dir / "config" / "preflight_checks.yaml"


def _parse_bool_text(value: str) -> bool:
    """Parse common launch-style boolean strings."""
    normalised = value.strip().lower()
    if normalised in TRUE_STRINGS:
        return True
    if normalised in FALSE_STRINGS:
        return False
    raise argparse.ArgumentTypeError(
        f"Expected a boolean value, got: {value}"
    )


def _strip_ros_cli_args(args: list[str] | None) -> list[str]:
    """Return only the CLI flags owned by this executable."""
    argv = sys.argv if args is None else ["preflight_check", *args]
    return remove_ros_args(args=argv)[1:]


def main(args: list[str] | None = None) -> None:
    """
    Run CLI entry point.

    Parse arguments, execute checks, print summary, and exit.
    """
    parser = _arg_parser()
    cli_args = parser.parse_args(args=_strip_ros_cli_args(args))

    rclpy.init(args=args)
    checker = None
    try:
        config_path = cli_args.config or _default_config_path()
        if not config_path.exists():
            raise FileNotFoundError(f"Preflight config not found: {config_path}")

        config = _load_yaml(config_path)
        checker = PreflightChecker(
            config,
            phase=cli_args.phase,
            use_sim_time=cli_args.use_sim_time,
        )
        results = checker.run()

        _print_summary(results)
        if cli_args.json_out is not None:
            _write_json(cli_args.json_out, results)

        raise SystemExit(_exit_code(results))
    except SystemExit:
        raise
    except Exception as exc:
        print(f"Preflight checker internal error: {exc}")
        raise SystemExit(EXIT_INTERNAL_ERROR) from exc
    finally:
        if checker is not None:
            checker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
