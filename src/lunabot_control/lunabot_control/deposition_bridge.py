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

"""Deposition hardware bridge for Cytron-driven linear actuators."""

from __future__ import annotations

import time
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool

from lunabot_interfaces.action import Deposit

_INHIBIT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_MAX_PHASES = 5
_MAX_PHASE_DURATION_S = 30.0


class DepositionBridge(Node):
    """Drive linear actuators for the hopper dump sequence."""

    def __init__(self) -> None:
        """Declare parameters and initialise hardware state."""
        super().__init__("deposition_bridge")
        self._declare_parameters()
        self._validate_parameters()
        self._init_state()
        self._init_gpio()
        self._init_ros()
        self.get_logger().info("Deposition bridge ready")

    def _declare_parameters(self) -> None:
        """Declare all configurable parameters."""
        self.declare_parameter("door_pwm_pin", 15)
        self.declare_parameter("door_dir_pin", 16)
        self.declare_parameter("bed_left_pwm_pin", 32)
        self.declare_parameter("bed_left_dir_pin", 36)
        self.declare_parameter("bed_right_pwm_pin", 33)
        self.declare_parameter("bed_right_dir_pin", 37)
        self.declare_parameter("pwm_frequency_hz", 1000)
        self.declare_parameter("actuator_duty_pct", 80.0)
        self.declare_parameter("door_open_duration_s", 3.0)
        self.declare_parameter("door_close_duration_s", 3.0)
        self.declare_parameter("bed_raise_duration_s", 5.0)
        self.declare_parameter("bed_lower_duration_s", 5.0)
        self.declare_parameter("dump_hold_duration_s", 3.0)
        self.declare_parameter("feedback_hz", 5.0)

    def _validate_parameters(self) -> None:
        """Read and validate parameters."""
        self._door_pwm_pin = self.get_parameter(
            "door_pwm_pin"
        ).value
        self._door_dir_pin = self.get_parameter(
            "door_dir_pin"
        ).value
        self._bed_l_pwm = self.get_parameter(
            "bed_left_pwm_pin"
        ).value
        self._bed_l_dir = self.get_parameter(
            "bed_left_dir_pin"
        ).value
        self._bed_r_pwm = self.get_parameter(
            "bed_right_pwm_pin"
        ).value
        self._bed_r_dir = self.get_parameter(
            "bed_right_dir_pin"
        ).value
        self._pwm_freq = self.get_parameter(
            "pwm_frequency_hz"
        ).value
        self._duty = self.get_parameter(
            "actuator_duty_pct"
        ).value
        self._door_open_t = self.get_parameter(
            "door_open_duration_s"
        ).value
        self._door_close_t = self.get_parameter(
            "door_close_duration_s"
        ).value
        self._bed_raise_t = self.get_parameter(
            "bed_raise_duration_s"
        ).value
        self._bed_lower_t = self.get_parameter(
            "bed_lower_duration_s"
        ).value
        self._dump_hold_t = self.get_parameter(
            "dump_hold_duration_s"
        ).value
        self._fb_period = 1.0 / self.get_parameter(
            "feedback_hz"
        ).value

        if not 0.0 < self._duty <= 100.0:
            raise ValueError(
                f"actuator_duty_pct must be (0, 100]: "
                f"{self._duty}"
            )

    def _init_state(self) -> None:
        """Initialise mutable state."""
        self._estop_active = False
        self._motion_inhibited = False
        self._gpio_available = False
        self._door_pwm: Any = None
        self._bed_l_pwm_obj: Any = None
        self._bed_r_pwm_obj: Any = None

    def _init_gpio(self) -> None:
        """Set up Jetson GPIO pins for Cytron PWM+DIR control."""
        try:
            import Jetson.GPIO as GPIO
        except (ImportError, RuntimeError) as exc:
            self.get_logger().warn(
                f"GPIO unavailable: {exc}. "
                f"Running in dry-run mode."
            )
            self._gpio_available = False
            return

        GPIO.setmode(GPIO.BOARD)

        for pin in (
            self._door_dir_pin,
            self._bed_l_dir,
            self._bed_r_dir,
        ):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        for pin in (
            self._door_pwm_pin,
            self._bed_l_pwm,
            self._bed_r_pwm,
        ):
            GPIO.setup(pin, GPIO.OUT)

        self._door_pwm = GPIO.PWM(
            self._door_pwm_pin, self._pwm_freq
        )
        self._bed_l_pwm_obj = GPIO.PWM(
            self._bed_l_pwm, self._pwm_freq
        )
        self._bed_r_pwm_obj = GPIO.PWM(
            self._bed_r_pwm, self._pwm_freq
        )
        self._door_pwm.start(0)
        self._bed_l_pwm_obj.start(0)
        self._bed_r_pwm_obj.start(0)

        self._gpio_available = True
        self.get_logger().info("GPIO initialised for Cytron control")

    def _init_ros(self) -> None:
        """Set up action server and safety subscriptions."""
        self._inhibit_sub = self.create_subscription(
            Bool,
            "/safety/motion_inhibit",
            self._inhibit_cb,
            _INHIBIT_QOS,
        )
        self._estop_sub = self.create_subscription(
            Bool, "/safety/estop", self._estop_cb, 10
        )
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            Deposit,
            "/mission/deposit",
            execute_callback=self._execute_deposit,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

    def _inhibit_cb(self, msg: Bool) -> None:
        """Track motion inhibit."""
        self._motion_inhibited = msg.data

    def _estop_cb(self, msg: Bool) -> None:
        """Track E-stop and halt actuators immediately."""
        self._estop_active = msg.data
        if msg.data:
            self._all_stop()

    def _goal_cb(self, _goal_request) -> GoalResponse:
        """Accept goals only when safe."""
        if self._estop_active or self._motion_inhibited:
            self.get_logger().warn(
                "Deposit goal rejected: safety active"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_cb(_goal_handle) -> CancelResponse:
        """Accept all cancel requests."""
        return CancelResponse.ACCEPT

    def _all_stop(self) -> None:
        """Immediately stop all actuators."""
        if not self._gpio_available:
            return
        for pwm_obj in (
            self._door_pwm,
            self._bed_l_pwm_obj,
            self._bed_r_pwm_obj,
        ):
            if pwm_obj is not None:
                pwm_obj.ChangeDutyCycle(0)

    def _set_actuator(
        self,
        pwm_obj: Any,
        dir_pin: int,
        extend: bool,
        duty: float,
    ) -> None:
        """Drive one actuator at given duty and direction."""
        if not self._gpio_available:
            return
        import Jetson.GPIO as GPIO
        direction = GPIO.HIGH if extend else GPIO.LOW
        GPIO.output(dir_pin, direction)
        pwm_obj.ChangeDutyCycle(duty)

    def _check_safety(self) -> str | None:
        """Return a failure reason if unsafe, else None."""
        if self._estop_active:
            return "E-stop active during deposition"
        if self._motion_inhibited:
            return "Motion inhibited during deposition"
        return None

    def _wait_phase(
        self,
        goal_handle,
        phase: int,
        duration_s: float,
        door_open: bool,
        bed_raised: bool,
        start_time: float,
    ) -> Deposit.Result | None:
        """Wait bounded duration, publishing feedback."""
        phase_start = time.monotonic()
        deadline = phase_start + min(
            duration_s, _MAX_PHASE_DURATION_S
        )

        while time.monotonic() < deadline and rclpy.ok():
            safety_reason = self._check_safety()
            if safety_reason is not None:
                self._all_stop()
                goal_handle.abort()
                return self._result(
                    False,
                    Deposit.Result.REASON_ESTOP,
                    safety_reason,
                    time.monotonic() - start_time,
                )

            if goal_handle.is_cancel_requested:
                self._all_stop()
                goal_handle.canceled()
                return self._result(
                    False,
                    Deposit.Result.REASON_CANCELED,
                    "Deposit canceled",
                    time.monotonic() - start_time,
                )

            fb = Deposit.Feedback()
            fb.phase = phase
            fb.elapsed_s = time.monotonic() - start_time
            fb.actuator_current_a = 0.0
            fb.door_open = door_open
            fb.bed_raised = bed_raised
            fb.estop_active = self._estop_active
            goal_handle.publish_feedback(fb)

            time.sleep(self._fb_period)

        return None

    @staticmethod
    def _result(
        success: bool,
        reason_code: int,
        reason: str,
        duration_s: float,
    ) -> Deposit.Result:
        """Build a Deposit.Result message."""
        r = Deposit.Result()
        r.success = success
        r.reason_code = reason_code
        r.failure_reason = reason
        r.residual_fill_fraction_estimate = (
            0.05 if success else 1.0
        )
        r.duration_s = duration_s
        return r

    def _execute_deposit(self, goal_handle) -> Deposit.Result:
        """Run the full dump sequence: open → raise → hold → lower → close."""
        start = time.monotonic()
        self.get_logger().info("Deposit sequence starting")

        # Phase 1: Open door
        self._set_actuator(
            self._door_pwm, self._door_dir_pin,
            extend=True, duty=self._duty,
        )
        result = self._wait_phase(
            goal_handle,
            Deposit.Feedback.PHASE_OPENING,
            self._door_open_t,
            door_open=True, bed_raised=False,
            start_time=start,
        )
        if result is not None:
            return result
        self._set_actuator(
            self._door_pwm, self._door_dir_pin,
            extend=False, duty=0,
        )

        # Phase 2: Raise bed
        for pwm_obj, dir_pin in (
            (self._bed_l_pwm_obj, self._bed_l_dir),
            (self._bed_r_pwm_obj, self._bed_r_dir),
        ):
            self._set_actuator(
                pwm_obj, dir_pin,
                extend=True, duty=self._duty,
            )
        result = self._wait_phase(
            goal_handle,
            Deposit.Feedback.PHASE_RAISING,
            self._bed_raise_t,
            door_open=True, bed_raised=True,
            start_time=start,
        )
        if result is not None:
            return result

        # Phase 3: Hold for dumping
        result = self._wait_phase(
            goal_handle,
            Deposit.Feedback.PHASE_DUMPING,
            self._dump_hold_t,
            door_open=True, bed_raised=True,
            start_time=start,
        )
        if result is not None:
            return result

        # Phase 4: Lower bed
        for pwm_obj, dir_pin in (
            (self._bed_l_pwm_obj, self._bed_l_dir),
            (self._bed_r_pwm_obj, self._bed_r_dir),
        ):
            self._set_actuator(
                pwm_obj, dir_pin,
                extend=False, duty=self._duty,
            )
        result = self._wait_phase(
            goal_handle,
            Deposit.Feedback.PHASE_CLOSING,
            self._bed_lower_t,
            door_open=True, bed_raised=False,
            start_time=start,
        )
        if result is not None:
            return result
        self._all_stop()

        # Phase 5: Close door
        self._set_actuator(
            self._door_pwm, self._door_dir_pin,
            extend=False, duty=self._duty,
        )
        result = self._wait_phase(
            goal_handle,
            Deposit.Feedback.PHASE_CLOSING,
            self._door_close_t,
            door_open=False, bed_raised=False,
            start_time=start,
        )
        if result is not None:
            return result
        self._all_stop()

        elapsed = time.monotonic() - start
        self.get_logger().info(
            f"Deposit complete in {elapsed:.1f}s"
        )
        goal_handle.succeed()
        return self._result(
            True,
            Deposit.Result.REASON_SUCCESS,
            "",
            elapsed,
        )

    def destroy_node(self) -> None:
        """Stop actuators and clean up GPIO."""
        self._all_stop()
        if self._gpio_available:
            import Jetson.GPIO as GPIO
            GPIO.cleanup()
        self._action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the deposition_bridge executable."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = DepositionBridge()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
