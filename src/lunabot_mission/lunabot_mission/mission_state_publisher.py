"""Mission state publisher for competition observability.

Publishes live mission state so operators can monitor robot progress:
  /mission/state             — current phase (String)
  /mission/autonomy_mode     — AUTONOMY or TELEOP (String)
  /mission/time_remaining_s  — competition countdown (Float32)
  /mission/cycle_count       — completed excavate→deposit cycles (Int32)
  /mission/last_failure_reason — last abort reason string (String)

State is inferred from action server status topics published by the
Nav2 BT executor when it calls /mission/excavate and /mission/deposit.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import Float32, Int32, String

STATE_PRECHECK = "PRECHECK"
STATE_AUTONOMY_ACTIVE = "AUTONOMY_ACTIVE"
STATE_SAFE_STOP = "SAFE_STOP"
STATE_RUN_COMPLETE = "RUN_COMPLETE"

# Action status topics use RELIABLE + TRANSIENT_LOCAL
_ACTION_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MissionStatePublisher(Node):
    """Publish mission observability topics derived from action server status."""

    def __init__(self):
        """Initialise subscriptions, publishers, and state."""
        super().__init__("mission_state_publisher")

        self.declare_parameter("competition_duration_s", 900.0)

        self._state = STATE_PRECHECK
        self._cycle_count = 0
        self._last_failure_reason = ""
        self._competition_duration_s = float(
            self.get_parameter("competition_duration_s").value
        )
        self._elapsed_s = 0.0

        # Track whether each action is currently executing
        self._excavate_executing = False
        self._deposit_executing = False

        # Remember previous deposit statuses to detect new successes
        self._prev_deposit_statuses: dict[str, int] = {}

        self.create_subscription(
            GoalStatusArray,
            "/mission/excavate/_action/status",
            self._excavate_status_cb,
            _ACTION_STATUS_QOS,
        )
        self.create_subscription(
            GoalStatusArray,
            "/mission/deposit/_action/status",
            self._deposit_status_cb,
            _ACTION_STATUS_QOS,
        )

        self._pub_state = self.create_publisher(String, "/mission/state", 10)
        self._pub_mode = self.create_publisher(String, "/mission/autonomy_mode", 10)
        self._pub_time = self.create_publisher(Float32, "/mission/time_remaining_s", 10)
        self._pub_cycles = self.create_publisher(Int32, "/mission/cycle_count", 10)
        self._pub_failure = self.create_publisher(
            String, "/mission/last_failure_reason", 10
        )

        self.create_timer(1.0, self._tick)

        self.get_logger().info("Mission state publisher online")

    def _excavate_status_cb(self, msg: GoalStatusArray) -> None:
        """Update excavation execution flag and detect aborts."""
        self._excavate_executing = any(
            s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list
        )
        for s in msg.status_list:
            if s.status == GoalStatus.STATUS_ABORTED:
                self._last_failure_reason = "excavate aborted"
                if self._state == STATE_AUTONOMY_ACTIVE:
                    self._state = STATE_SAFE_STOP

    def _deposit_status_cb(self, msg: GoalStatusArray) -> None:
        """Detect completed deposit cycles and aborts."""
        current: dict[str, int] = {
            str(bytes(s.goal_info.goal_id.uuid)): s.status for s in msg.status_list
        }

        for goal_id, status in current.items():
            prev = self._prev_deposit_statuses.get(goal_id)
            if (
                status == GoalStatus.STATUS_SUCCEEDED
                and prev != GoalStatus.STATUS_SUCCEEDED
            ):
                self._cycle_count += 1
                self.get_logger().info(
                    f"Cycle complete — total cycles: {self._cycle_count}"
                )

        for s in msg.status_list:
            if s.status == GoalStatus.STATUS_ABORTED:
                self._last_failure_reason = "deposit aborted"
                if self._state == STATE_AUTONOMY_ACTIVE:
                    self._state = STATE_SAFE_STOP

        self._deposit_executing = any(
            s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list
        )
        self._prev_deposit_statuses = current

    def _tick(self) -> None:
        """Advance state machine and publish all observability topics."""
        if self._state == STATE_PRECHECK:
            if self._excavate_executing or self._deposit_executing:
                self._state = STATE_AUTONOMY_ACTIVE
                self.get_logger().info("State → AUTONOMY_ACTIVE")

        if self._state == STATE_AUTONOMY_ACTIVE:
            self._elapsed_s += 1.0
            if self._elapsed_s >= self._competition_duration_s:
                self._state = STATE_RUN_COMPLETE
                self.get_logger().info("Competition time elapsed — State → RUN_COMPLETE")

        state_msg = String()
        state_msg.data = self._state
        self._pub_state.publish(state_msg)

        mode_msg = String()
        mode_msg.data = "AUTONOMY"
        self._pub_mode.publish(mode_msg)

        time_msg = Float32()
        time_msg.data = float(
            max(0.0, self._competition_duration_s - self._elapsed_s)
        )
        self._pub_time.publish(time_msg)

        cycles_msg = Int32()
        cycles_msg.data = self._cycle_count
        self._pub_cycles.publish(cycles_msg)

        failure_msg = String()
        failure_msg.data = self._last_failure_reason
        self._pub_failure.publish(failure_msg)


def main(args=None):
    """Start the mission state publisher and spin until interrupted."""
    rclpy.init(args=args)
    node = MissionStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
