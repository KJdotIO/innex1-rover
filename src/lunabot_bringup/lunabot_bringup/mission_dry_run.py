"""One-shot mission dry-run harness for the simulation stack."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from math import cos, sin
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from lunabot_bringup.preflight_check import PreflightChecker, _load_yaml
from lunabot_bringup.preflight_check import _exit_code as preflight_exit_code
from lunabot_bringup.preflight_check import _print_summary as print_preflight_summary
from lunabot_bringup.preflight_profiles import PHASE_RUNTIME
from lunabot_interfaces.action import Deposit, Excavate

EXIT_SUCCESS = 0
EXIT_FAILURE = 1
STEP_ORDER = ("travel", "excavate", "deposit")


@dataclass(frozen=True)
class StepStatus:
    """Terminal status for one dry-run phase."""

    name: str
    passed: bool
    detail: str


def execute_dry_run(
    step_runners: list[tuple[str, Callable[[], tuple[bool, str]]]],
    *,
    preflight_runner: Callable[[], tuple[bool, str]] | None = None,
) -> list[StepStatus]:
    """Run the preflight gate and then each step once."""
    if preflight_runner is not None:
        preflight_passed, preflight_detail = preflight_runner()
        if not preflight_passed:
            return [
                StepStatus(name=name, passed=False, detail=preflight_detail)
                for name, _runner in step_runners
            ]

    results: list[StepStatus] = []

    for index, (name, runner) in enumerate(step_runners):
        passed, detail = runner()
        results.append(StepStatus(name=name, passed=passed, detail=detail))
        if passed:
            continue

        for remaining_name, _remaining_runner in step_runners[index + 1:]:
            results.append(
                StepStatus(
                    name=remaining_name,
                    passed=False,
                    detail=f"not run because {name} failed",
                )
            )
        break

    return results


def overall_passed(results: list[StepStatus]) -> bool:
    """Return True when every dry-run phase passed."""
    return bool(results) and all(result.passed for result in results)


def dry_run_exit_code(results: list[StepStatus]) -> int:
    """Return the process exit code for a dry-run result set."""
    if overall_passed(results):
        return EXIT_SUCCESS
    return EXIT_FAILURE


def summary_lines(results: list[StepStatus]) -> list[str]:
    """Build flat summary lines for dry-run output."""
    status_by_name = {result.name: result.passed for result in results}
    lines = [
        f"{name}: {'pass' if status_by_name.get(name, False) else 'fail'}"
        for name in STEP_ORDER
    ]
    lines.append(f"overall: {'pass' if overall_passed(results) else 'fail'}")
    return lines


def _goal_status_text(status: int) -> str:
    """Return a readable action status string."""
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }
    return mapping.get(status, f"status={status}")


class MissionDryRunHarness(Node):
    """Run one bounded travel, excavation, and deposit sequence."""

    def __init__(self) -> None:
        super().__init__("mission_dry_run_harness")

        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("runtime_preflight_enabled", True)
        self.declare_parameter(
            "preflight_config",
            str(self._default_preflight_config_path()),
        )
        self.declare_parameter("action_wait_timeout_s", 30.0)
        self.declare_parameter("travel_result_timeout_s", 60.0)
        self.declare_parameter("result_timeout_padding_s", 5.0)
        self.declare_parameter("travel_frame_id", "map")
        self.declare_parameter("travel_x_m", 0.5)
        self.declare_parameter("travel_y_m", 0.0)
        self.declare_parameter("travel_yaw_rad", 0.0)
        self.declare_parameter("excavate_timeout_s", 12.0)
        self.declare_parameter("excavate_target_fill_fraction", 0.8)
        self.declare_parameter("excavate_max_drive_speed_mps", 0.2)
        self.declare_parameter("deposit_timeout_s", 10.0)
        self.declare_parameter("deposit_dump_duration_s", 3.0)
        self.declare_parameter("deposit_require_close_after_dump", True)

        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose_gate",
        )
        self._excavate_client = ActionClient(
            self,
            Excavate,
            "/mission/excavate",
        )
        self._deposit_client = ActionClient(
            self,
            Deposit,
            "/mission/deposit",
        )

    @staticmethod
    def _default_preflight_config_path() -> Path:
        """Return the installed dry-run preflight config path."""
        share_dir = Path(get_package_share_directory("lunabot_bringup"))
        return share_dir / "config" / "preflight_checks_dry_run.yaml"

    @staticmethod
    def _as_bool(value) -> bool:
        """Return a sensible bool from launch-style values."""
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def run(self) -> int:
        """Execute the one-shot dry run and print a flat summary."""
        results = execute_dry_run(
            [
                ("travel", self._run_travel_step),
                ("excavate", self._run_excavate_step),
                ("deposit", self._run_deposit_step),
            ],
            preflight_runner=self._run_runtime_preflight,
        )

        for line in summary_lines(results):
            print(line)

        return dry_run_exit_code(results)

    def _action_wait_timeout_s(self) -> float:
        """Return the common action server discovery timeout."""
        return float(self.get_parameter("action_wait_timeout_s").value)

    def _result_timeout_padding_s(self) -> float:
        """Return the extra wait added beyond action goal deadlines."""
        return float(self.get_parameter("result_timeout_padding_s").value)

    def _run_runtime_preflight(self) -> tuple[bool, str]:
        """Run the existing runtime preflight profile before the dry run."""
        if not self._as_bool(self.get_parameter("runtime_preflight_enabled").value):
            detail = "runtime preflight skipped"
            self.get_logger().info(detail)
            return True, detail

        config_path = Path(str(self.get_parameter("preflight_config").value))
        checker = None

        try:
            if not config_path.exists():
                detail = f"preflight config not found: {config_path}"
                self.get_logger().error(detail)
                return False, detail

            config = _load_yaml(config_path)
            checker = PreflightChecker(
                config,
                phase=PHASE_RUNTIME,
                use_sim_time=self._as_bool(self.get_parameter("use_sim_time").value),
            )
            results = checker.run()
            print_preflight_summary(results)

            if preflight_exit_code(results) == 0:
                detail = "runtime preflight passed"
                self.get_logger().info(detail)
                return True, detail

            detail = "runtime preflight failed"
            self.get_logger().error(detail)
            return False, detail
        except (OSError, ValueError, RuntimeError) as exc:
            detail = f"runtime preflight error: {exc}"
            self.get_logger().error(detail)
            return False, detail
        finally:
            if checker is not None:
                checker.destroy_node()

    def _wait_for_server(
        self,
        client: ActionClient,
        action_name: str,
    ) -> tuple[bool, str]:
        """Wait for one action server before sending a goal."""
        if client.wait_for_server(timeout_sec=self._action_wait_timeout_s()):
            return True, f"{action_name} server available"
        return False, f"{action_name} server unavailable"

    def _wait_for_future(
        self,
        future,
        timeout_s: float,
        timeout_detail: str,
    ):
        """Wait for one future and return its result when ready."""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if future.done():
            return future.result()
        return timeout_detail

    def _send_goal(
        self,
        client: ActionClient,
        goal,
        *,
        timeout_detail: str,
    ):
        """Send one goal and return the accepted goal handle."""
        send_goal_future = client.send_goal_async(goal)
        return self._wait_for_future(
            send_goal_future,
            self._action_wait_timeout_s(),
            timeout_detail,
        )

    def _accepted_goal_handle(
        self,
        goal_handle,
        *,
        rejection_detail: str,
    ):
        """Return the accepted goal handle or a rejection detail string."""
        if isinstance(goal_handle, str):
            return goal_handle
        if goal_handle is None or not goal_handle.accepted:
            return rejection_detail
        return goal_handle

    def _wait_for_goal_result(
        self,
        goal_handle,
        *,
        timeout_s: float,
        timeout_detail: str,
    ):
        """Wait for one goal result and return the wrapped result."""
        result_future = goal_handle.get_result_async()
        return self._wait_for_future(result_future, timeout_s, timeout_detail)

    def _goal_status_detail(self, action_name: str, status: int) -> str:
        """Return a readable action status detail string."""
        return f"{action_name} finished with {_goal_status_text(status)} status"

    def _run_travel_step(self) -> tuple[bool, str]:
        """Send one bounded travel goal through the gate action."""
        self.get_logger().info("Starting travel phase")
        available, detail = self._wait_for_server(
            self._navigate_client,
            "/navigate_to_pose_gate",
        )
        if not available:
            self.get_logger().error(detail)
            return False, detail

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self.get_parameter("travel_frame_id").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter("travel_x_m").value)
        goal.pose.pose.position.y = float(self.get_parameter("travel_y_m").value)

        yaw = float(self.get_parameter("travel_yaw_rad").value)
        goal.pose.pose.orientation.z = sin(yaw / 2.0)
        goal.pose.pose.orientation.w = cos(yaw / 2.0)

        goal_handle = self._send_goal(
            self._navigate_client,
            goal,
            timeout_detail="travel goal response timed out",
        )
        accepted_goal_handle = self._accepted_goal_handle(
            goal_handle,
            rejection_detail="travel goal rejected",
        )
        if isinstance(accepted_goal_handle, str):
            self.get_logger().error(accepted_goal_handle)
            return False, accepted_goal_handle

        result = self._wait_for_goal_result(
            accepted_goal_handle,
            timeout_s=float(self.get_parameter("travel_result_timeout_s").value),
            timeout_detail="travel result timed out",
        )
        if isinstance(result, str):
            self.get_logger().error(result)
            return False, result
        if result is None:
            detail = "travel result unavailable"
            self.get_logger().error(detail)
            return False, detail
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            detail = self._goal_status_detail("travel", result.status)
            self.get_logger().error(detail)
            return False, detail

        detail = "travel goal succeeded"
        self.get_logger().info(detail)
        return True, detail

    def _run_excavate_step(self) -> tuple[bool, str]:
        """Send one bounded excavation goal."""
        self.get_logger().info("Starting excavation phase")
        available, detail = self._wait_for_server(
            self._excavate_client,
            "/mission/excavate",
        )
        if not available:
            self.get_logger().error(detail)
            return False, detail

        goal = Excavate.Goal()
        goal.mode = Excavate.Goal.MODE_AUTO
        goal.timeout_s = float(self.get_parameter("excavate_timeout_s").value)
        goal.target_fill_fraction = float(
            self.get_parameter("excavate_target_fill_fraction").value
        )
        goal.max_drive_speed_mps = float(
            self.get_parameter("excavate_max_drive_speed_mps").value
        )

        goal_handle = self._send_goal(
            self._excavate_client,
            goal,
            timeout_detail="excavate goal response timed out",
        )
        accepted_goal_handle = self._accepted_goal_handle(
            goal_handle,
            rejection_detail="excavate goal rejected",
        )
        if isinstance(accepted_goal_handle, str):
            self.get_logger().error(accepted_goal_handle)
            return False, accepted_goal_handle

        result = self._wait_for_goal_result(
            accepted_goal_handle,
            timeout_s=goal.timeout_s + self._result_timeout_padding_s(),
            timeout_detail="excavate result timed out",
        )
        if isinstance(result, str):
            self.get_logger().error(result)
            return False, result
        if result is None:
            detail = "excavate result unavailable"
            self.get_logger().error(detail)
            return False, detail
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            detail = self._goal_status_detail("excavate", result.status)
            self.get_logger().error(detail)
            return False, detail
        if not result.result.success:
            detail = (
                f"excavate failed ({result.result.reason_code}): "
                f"{result.result.failure_reason}"
            )
            self.get_logger().error(detail)
            return False, detail

        detail = "excavate goal succeeded"
        self.get_logger().info(detail)
        return True, detail

    def _run_deposit_step(self) -> tuple[bool, str]:
        """Send one bounded deposit goal."""
        self.get_logger().info("Starting deposit phase")
        available, detail = self._wait_for_server(
            self._deposit_client,
            "/mission/deposit",
        )
        if not available:
            self.get_logger().error(detail)
            return False, detail

        goal = Deposit.Goal()
        goal.mode = Deposit.Goal.MODE_AUTO
        goal.timeout_s = float(self.get_parameter("deposit_timeout_s").value)
        goal.dump_duration_s = float(
            self.get_parameter("deposit_dump_duration_s").value
        )
        goal.require_close_after_dump = bool(
            self.get_parameter("deposit_require_close_after_dump").value
        )

        goal_handle = self._send_goal(
            self._deposit_client,
            goal,
            timeout_detail="deposit goal response timed out",
        )
        goal_handle = self._accepted_goal_handle(
            goal_handle,
            rejection_detail="deposit goal rejected",
        )
        if isinstance(goal_handle, str):
            self.get_logger().error(goal_handle)
            return False, goal_handle

        result = self._wait_for_goal_result(
            goal_handle,
            timeout_s=goal.timeout_s + self._result_timeout_padding_s(),
            timeout_detail="deposit result timed out",
        )
        if isinstance(result, str):
            self.get_logger().error(result)
            return False, result
        if result is None:
            detail = "deposit result unavailable"
            self.get_logger().error(detail)
            return False, detail
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            detail = self._goal_status_detail("deposit", result.status)
            self.get_logger().error(detail)
            return False, detail
        if not result.result.success:
            detail = (
                f"deposit failed ({result.result.reason_code}): "
                f"{result.result.failure_reason}"
            )
            self.get_logger().error(detail)
            return False, detail

        detail = "deposit goal succeeded"
        self.get_logger().info(detail)
        return True, detail


def main(args=None) -> None:
    """Run the mission dry-run harness once and exit."""
    rclpy.init(args=args)
    node = None

    try:
        node = MissionDryRunHarness()
        raise SystemExit(node.run())
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
