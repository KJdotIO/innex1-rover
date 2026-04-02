"""Gate NavigateToPose goals until start-zone localisation is ready."""

from __future__ import annotations

import time

from action_msgs.msg import GoalStatus
from lunabot_bringup.localisation_readiness import is_localisation_ready
from lunabot_interfaces.msg import LocalisationStartZoneStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.node import Node


def _parse_bool(value) -> bool:
    """Return a sensible bool from launch-provided parameter values."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class NavigateToPoseGate(Node):
    """Expose an autonomy-only NavigateToPose action gated by localisation."""

    def __init__(self) -> None:
        super().__init__("navigate_to_pose_gate")

        self.declare_parameter("status_topic", "/localisation/start_zone_status")
        self.declare_parameter("public_action_name", "/navigate_to_pose_gate")
        self.declare_parameter("internal_action_name", "/navigate_to_pose")
        self.declare_parameter("readiness_timeout_s", 5.0)
        self.declare_parameter("hazard_topic", "/terrain_hazard/grid")
        self.declare_parameter("hazard_timeout_s", 2.0)
        self.declare_parameter("gate_enabled", True)

        self.status_topic = self.get_parameter("status_topic").value
        self.public_action_name = self.get_parameter("public_action_name").value
        self.internal_action_name = self.get_parameter("internal_action_name").value
        self.hazard_topic = self.get_parameter("hazard_topic").value
        self.readiness_timeout_ns = int(
            float(self.get_parameter("readiness_timeout_s").value) * 1e9
        )
        self.hazard_timeout_ns = int(
            float(self.get_parameter("hazard_timeout_s").value) * 1e9
        )
        self.gate_enabled = _parse_bool(self.get_parameter("gate_enabled").value)

        self._latest_status = None
        self._latest_hazard = None

        self.create_subscription(
            LocalisationStartZoneStatus,
            self.status_topic,
            self._on_status,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.hazard_topic,
            self._on_hazard,
            10,
        )

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            self.internal_action_name,
        )
        # Humble cannot reliably hide or rename Nav2's own action server via
        # remapping, so this node exposes a separate action surface that the
        # rover's autonomy stack must call explicitly.
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            self.public_action_name,
            execute_callback=self._execute_goal,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )

        self.get_logger().info(
            "NavigateToPose readiness gate active on "
            f"{self.public_action_name}; native Nav2 action "
            f"{self.internal_action_name} remains available for tooling."
        )

    def _on_status(self, msg: LocalisationStartZoneStatus) -> None:
        """Cache the latest localisation readiness summary."""
        self._latest_status = msg

    def _on_hazard(self, msg: OccupancyGrid) -> None:
        """Cache the latest terrain hazard grid."""
        self._latest_hazard = msg

    def _hazard_ready_for_travel(self) -> bool:
        """Return whether the latest terrain hazard grid is still fresh."""
        if self._latest_hazard is None:
            return False

        stamp = self._latest_hazard.header.stamp
        hazard_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        if hazard_ns <= 0 or now_ns <= 0 or now_ns < hazard_ns:
            return False
        return (now_ns - hazard_ns) <= self.hazard_timeout_ns

    def _ready_for_travel(self) -> bool:
        """Return whether the latest localisation status permits travel goals."""
        if not self.gate_enabled:
            return True
        now_ns = self.get_clock().now().nanoseconds
        return is_localisation_ready(
            self._latest_status,
            None,
            now_ns,
            self.readiness_timeout_ns,
        ) and self._hazard_ready_for_travel()

    def _on_goal(self, _goal) -> GoalResponse:
        """Accept travel goals only when localisation is ready and Nav2 is up."""
        if not self._ready_for_travel():
            self.get_logger().warn(
                "Rejecting NavigateToPose goal because localisation or terrain "
                "hazard readiness is not ready."
            )
            return GoalResponse.REJECT

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                "Rejecting NavigateToPose goal because the internal Nav2 action "
                "server is unavailable."
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        """Accept cancellation requests and mirror them to Nav2."""
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle) -> None:
        """Forward the accepted goal to Nav2 and mirror the result."""
        send_goal_future = self._action_client.send_goal_async(
            goal_handle.request,
            feedback_callback=lambda feedback: goal_handle.publish_feedback(
                feedback.feedback
            ),
        )

        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.05)

        client_goal_handle = send_goal_future.result()
        if client_goal_handle is None or not client_goal_handle.accepted:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return NavigateToPose.Result()

        result_future = client_goal_handle.get_result_async()
        cancel_future = None

        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested and cancel_future is None:
                cancel_future = client_goal_handle.cancel_goal_async()
            time.sleep(0.05)

        wrapped_result = result_future.result()
        result = wrapped_result.result
        status = wrapped_result.status

        if goal_handle.is_cancel_requested or status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        elif status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result


def main(args=None) -> None:
    """Run the NavigateToPose gate node."""
    rclpy.init(args=args)
    node = NavigateToPoseGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # pragma: no cover - shutdown can already be in progress
            pass
