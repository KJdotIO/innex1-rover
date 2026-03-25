"""Excavation action adapter."""

from time import monotonic, sleep

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from lunabot_interfaces.action import Excavate
from lunabot_interfaces.msg import ExcavationStatus, ExcavationTelemetry


STATE_TO_PHASE = {
    ExcavationStatus.STATE_IDLE: Excavate.Feedback.PHASE_PRECHECK,
    ExcavationStatus.STATE_HOMING: Excavate.Feedback.PHASE_PRECHECK,
    ExcavationStatus.STATE_READY: Excavate.Feedback.PHASE_PRECHECK,
    ExcavationStatus.STATE_STARTING: Excavate.Feedback.PHASE_SPINUP,
    ExcavationStatus.STATE_EXCAVATING: Excavate.Feedback.PHASE_DIGGING,
    ExcavationStatus.STATE_STOPPING: Excavate.Feedback.PHASE_RETRACT,
    ExcavationStatus.STATE_FAULT: Excavate.Feedback.PHASE_RETRACT,
}

ACTIVE_STATES = {
    ExcavationStatus.STATE_HOMING,
    ExcavationStatus.STATE_STARTING,
    ExcavationStatus.STATE_EXCAVATING,
    ExcavationStatus.STATE_STOPPING,
}


class ExcavationActionServer(Node):
    """Bridge the mission-facing excavation action onto the controller."""

    def __init__(self):
        """Initialise action server, status subscriptions, and service clients."""
        super().__init__("excavation_action_server")

        self.declare_parameter("nominal_duration_s", 8.0)
        self.declare_parameter("loop_period_s", 0.2)
        self.declare_parameter("stop_timeout_s", 2.0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._status = None
        self._latest_telemetry = None
        self._callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            ExcavationStatus,
            "/excavation/status",
            self._handle_status,
            qos,
        )
        self.create_subscription(
            ExcavationTelemetry,
            "/excavation/telemetry",
            self._handle_telemetry,
            qos,
        )

        self._home_client = self.create_client(Trigger, "/excavation/home")
        self._start_client = self.create_client(Trigger, "/excavation/start")
        self._stop_client = self.create_client(Trigger, "/excavation/stop")

        self._server = ActionServer(
            self,
            Excavate,
            "/mission/excavate",
            execute_callback=self.execute_excavate,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

    def destroy_node(self):
        """Destroy the action server before node teardown."""
        self._server.destroy()
        super().destroy_node()

    def _handle_status(self, msg: ExcavationStatus):
        """Store the latest excavation controller status."""
        self._status = msg

    def _handle_telemetry(self, msg: ExcavationTelemetry):
        """Store the latest excavation telemetry sample."""
        self._latest_telemetry = msg

    def goal_callback(self, _goal_request):
        """Accept excavation goals for controller-backed execution."""
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        """Accept cancellation so the mechanism can be stopped safely."""
        return CancelResponse.ACCEPT

    @property
    def loop_period_s(self):
        """Return the action feedback loop period."""
        return float(self.get_parameter("loop_period_s").value)

    def _result(self, success: bool, reason_code: int, reason: str, duration_s: float):
        """Build an excavation result."""
        result = Excavate.Result()
        result.success = bool(success)
        result.reason_code = int(reason_code)
        result.failure_reason = str(reason)
        result.collected_mass_kg_estimate = 0.0
        result.duration_s = float(duration_s)
        return result

    def _call_trigger(self, client, timeout_s: float):
        """Call a Trigger service synchronously and return the response."""
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            return None
        return future.result()

    def _publish_feedback(self, goal_handle, elapsed: float):
        """Publish action feedback from controller status and telemetry."""
        telemetry = self._latest_telemetry
        state = (
            self._status.state
            if self._status is not None
            else ExcavationStatus.STATE_IDLE
        )

        feedback = Excavate.Feedback()
        feedback.phase = STATE_TO_PHASE.get(state, Excavate.Feedback.PHASE_PRECHECK)
        feedback.elapsed_s = float(elapsed)
        feedback.fill_fraction_estimate = 0.0
        feedback.excavation_motor_current_a = (
            float(telemetry.motor_current_a)
            if telemetry is not None
            else (
                float(self._status.motor_current_a) if self._status is not None else 0.0
            )
        )
        feedback.jam_detected = (
            self._status is not None
            and self._status.fault_code == ExcavationStatus.FAULT_OVERCURRENT
        )
        feedback.estop_active = (
            self._status.estop_active if self._status is not None else False
        )
        goal_handle.publish_feedback(feedback)

    def _fault_reason(self):
        """Map the latest status fault into an action result code and reason."""
        if self._status is None:
            return Excavate.Result.REASON_SHUTDOWN, "Excavation status unavailable"
        if self._status.fault_code == ExcavationStatus.FAULT_ESTOP:
            return Excavate.Result.REASON_ESTOP, "Excavation estop active"
        if self._status.fault_code == ExcavationStatus.FAULT_OVERCURRENT:
            return (
                Excavate.Result.REASON_JAM_OR_OVERCURRENT,
                "Excavation jam or overcurrent detected",
            )
        if self._status.fault_code == ExcavationStatus.FAULT_DRIVER:
            return Excavate.Result.REASON_DRIVER_FAULT, "Excavation driver fault active"
        return Excavate.Result.REASON_INTERLOCK_BLOCKED, "Excavation controller faulted"

    def _wait_for_ready_or_fault(self, timeout_s: float, goal_handle, start_time: float):
        """Wait until the controller is ready, faulted, canceled, or timed out."""
        while rclpy.ok():
            elapsed = monotonic() - start_time

            if goal_handle.is_cancel_requested:
                return "cancel", elapsed
            if timeout_s > 0.0 and elapsed >= timeout_s:
                return "timeout", elapsed
            if self._status is not None and self._status.state == ExcavationStatus.STATE_READY:
                return "ready", elapsed
            if self._status is not None and self._status.state == ExcavationStatus.STATE_FAULT:
                return "fault", elapsed

            self._publish_feedback(goal_handle, elapsed)
            sleep(self.loop_period_s)

        return "shutdown", monotonic() - start_time

    def _wait_for_stop_settle(self, timeout_s: float):
        """Wait until the controller leaves active motion states."""
        settle_start = monotonic()
        while rclpy.ok():
            if self._status is not None and self._status.state not in ACTIVE_STATES:
                return True
            if monotonic() - settle_start >= timeout_s:
                return False
            sleep(self.loop_period_s)
        return False

    def execute_excavate(self, goal_handle):
        """Run one bounded excavation attempt through the controller."""
        timeout_s = float(goal_handle.request.timeout_s)
        nominal_duration = float(self.get_parameter("nominal_duration_s").value)
        stop_timeout_s = float(self.get_parameter("stop_timeout_s").value)
        start = monotonic()
        saw_excavating = False
        stop_requested = False
        state = (
            self._status.state
            if self._status is not None
            else ExcavationStatus.STATE_IDLE
        )

        if state == ExcavationStatus.STATE_FAULT:
            goal_handle.abort()
            code, reason = self._fault_reason()
            return self._result(False, code, reason, 0.0)

        if state not in (
            ExcavationStatus.STATE_IDLE,
            ExcavationStatus.STATE_READY,
        ):
            goal_handle.abort()
            return self._result(
                False,
                Excavate.Result.REASON_INTERLOCK_BLOCKED,
                f"Excavation controller busy in state '{state}'",
                0.0,
            )

        if state != ExcavationStatus.STATE_READY:
            response = self._call_trigger(self._home_client, 2.0)
            if response is None or not response.success:
                goal_handle.abort()
                return self._result(
                    False,
                    Excavate.Result.REASON_INTERLOCK_BLOCKED,
                    "Excavation home command was rejected",
                    0.0,
                )

            homing_result, elapsed = self._wait_for_ready_or_fault(
                timeout_s,
                goal_handle,
                start,
            )
            if homing_result == "cancel":
                self._call_trigger(self._stop_client, stop_timeout_s)
                if not self._wait_for_stop_settle(stop_timeout_s):
                    goal_handle.abort()
                    return self._result(
                        False,
                        Excavate.Result.REASON_INTERLOCK_BLOCKED,
                        "Excavation did not settle after cancel during homing",
                        elapsed,
                    )
                goal_handle.canceled()
                return self._result(
                    False,
                    Excavate.Result.REASON_CANCELED,
                    "Excavation goal canceled during homing",
                    elapsed,
                )
            if homing_result == "timeout":
                self._call_trigger(self._stop_client, stop_timeout_s)
                if not self._wait_for_stop_settle(stop_timeout_s):
                    goal_handle.abort()
                    return self._result(
                        False,
                        Excavate.Result.REASON_INTERLOCK_BLOCKED,
                        "Excavation did not settle after homing timeout",
                        elapsed,
                    )
                goal_handle.abort()
                return self._result(
                    False,
                    Excavate.Result.REASON_TIMEOUT,
                    "Excavation timeout reached during homing",
                    elapsed,
                )
            if homing_result == "fault":
                goal_handle.abort()
                code, reason = self._fault_reason()
                return self._result(False, code, reason, elapsed)
            if homing_result == "shutdown":
                goal_handle.abort()
                return self._result(
                    False,
                    Excavate.Result.REASON_SHUTDOWN,
                    "Node shutdown during excavation homing",
                    elapsed,
                )

        start_response = self._call_trigger(self._start_client, 2.0)
        if start_response is None or not start_response.success:
            goal_handle.abort()
            return self._result(
                False,
                Excavate.Result.REASON_INTERLOCK_BLOCKED,
                "Excavation start command was rejected",
                monotonic() - start,
            )

        while rclpy.ok():
            elapsed = monotonic() - start

            if goal_handle.is_cancel_requested:
                self._call_trigger(self._stop_client, stop_timeout_s)
                if not self._wait_for_stop_settle(stop_timeout_s):
                    goal_handle.abort()
                    return self._result(
                        False,
                        Excavate.Result.REASON_INTERLOCK_BLOCKED,
                        "Excavation did not settle after cancel",
                        elapsed,
                    )
                goal_handle.canceled()
                return self._result(
                    False,
                    Excavate.Result.REASON_CANCELED,
                    "Excavation goal canceled by client",
                    elapsed,
                )

            if timeout_s > 0.0 and elapsed >= timeout_s:
                self._call_trigger(self._stop_client, stop_timeout_s)
                if not self._wait_for_stop_settle(stop_timeout_s):
                    goal_handle.abort()
                    return self._result(
                        False,
                        Excavate.Result.REASON_INTERLOCK_BLOCKED,
                        "Excavation did not settle after timeout",
                        elapsed,
                    )
                goal_handle.abort()
                return self._result(
                    False,
                    Excavate.Result.REASON_TIMEOUT,
                    "Excavation timeout reached",
                    elapsed,
                )

            if self._status is not None and self._status.state == ExcavationStatus.STATE_FAULT:
                goal_handle.abort()
                code, reason = self._fault_reason()
                return self._result(False, code, reason, elapsed)

            if (
                self._status is not None
                and self._status.state == ExcavationStatus.STATE_EXCAVATING
            ):
                saw_excavating = True

            if saw_excavating and elapsed >= nominal_duration and not stop_requested:
                self._call_trigger(self._stop_client, stop_timeout_s)
                stop_requested = True

            if saw_excavating and self._status is not None and self._status.state in (
                ExcavationStatus.STATE_READY,
                ExcavationStatus.STATE_IDLE,
            ):
                goal_handle.succeed()
                return self._result(True, Excavate.Result.REASON_SUCCESS, "", elapsed)

            self._publish_feedback(goal_handle, elapsed)
            sleep(self.loop_period_s)

        if goal_handle.is_active:
            goal_handle.abort()
        return self._result(
            False,
            Excavate.Result.REASON_SHUTDOWN,
            "Node shutdown during excavation",
            monotonic() - start,
        )


def main(args=None):
    """Run the excavation action server node."""
    rclpy.init(args=args)
    node = ExcavationActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
