"""Bench tooling for the excavation subsystem."""

import argparse
from threading import Event

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from lunabot_interfaces.msg import ExcavationStatus
from lunabot_interfaces.srv import ExcavationJog


STATE_LABELS = {
    ExcavationStatus.STATE_IDLE: "idle",
    ExcavationStatus.STATE_HOMING: "homing",
    ExcavationStatus.STATE_READY: "ready",
    ExcavationStatus.STATE_STARTING: "starting",
    ExcavationStatus.STATE_EXCAVATING: "excavating",
    ExcavationStatus.STATE_STOPPING: "stopping",
    ExcavationStatus.STATE_FAULT: "fault",
}

FAULT_LABELS = {
    ExcavationStatus.FAULT_NONE: "none",
    ExcavationStatus.FAULT_ESTOP: "estop",
    ExcavationStatus.FAULT_DRIVER: "driver",
    ExcavationStatus.FAULT_OVERCURRENT: "overcurrent",
    ExcavationStatus.FAULT_HOME_SWITCH_INVALID: "home_switch_invalid",
    ExcavationStatus.FAULT_COMMAND_REJECTED: "command_rejected",
}


class ExcavationBench(Node):
    """Provide a small CLI for excavation bench work."""

    def __init__(self):
        """Initialise service clients and one-shot status subscription."""
        super().__init__("excavation_bench")
        self._status_msg = None
        self._status_event = Event()

        self._home_client = self.create_client(Trigger, "/excavation/home")
        self._start_client = self.create_client(Trigger, "/excavation/start")
        self._stop_client = self.create_client(Trigger, "/excavation/stop")
        self._clear_fault_client = self.create_client(
            Trigger,
            "/excavation/clear_fault",
        )
        self._jog_client = self.create_client(
            ExcavationJog,
            "/excavation/jog_forward",
        )
        self.create_subscription(
            ExcavationStatus,
            "/excavation/status",
            self._handle_status,
            10,
        )

    def _handle_status(self, msg: ExcavationStatus):
        """Store the latest excavation status for one-shot inspection."""
        self._status_msg = msg
        self._status_event.set()

    def _call_trigger(self, client, timeout_s: float):
        """Call one Trigger service synchronously."""
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("service unavailable")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise RuntimeError("service call timed out")
        return future.result()

    def _call_jog(self, duration_s: float, timeout_s: float):
        """Call the bounded jog service synchronously."""
        if not self._jog_client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("service unavailable")
        request = ExcavationJog.Request()
        request.duration_s = float(duration_s)
        future = self._jog_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise RuntimeError("service call timed out")
        return future.result()

    def _wait_for_status(self, timeout_s: float):
        """Wait for one excavation status message."""
        self._status_event.clear()
        self._status_msg = None
        end_time = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._status_event.is_set():
                return self._status_msg
        raise RuntimeError("status timed out")


def _build_parser():
    """Build the excavation bench CLI parser."""
    parser = argparse.ArgumentParser(description="Excavation bench tooling")
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="service or status timeout in seconds",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("status", help="print one excavation status sample")
    subparsers.add_parser("home", help="request excavation homing")
    subparsers.add_parser("start", help="request excavation start")
    subparsers.add_parser("stop", help="request excavation stop")
    subparsers.add_parser("clear-fault", help="clear a latched excavation fault")

    jog_parser = subparsers.add_parser(
        "jog-forward",
        help="request a bounded forward jog",
    )
    jog_parser.add_argument(
        "--duration",
        type=float,
        required=True,
        help="forward jog duration in seconds",
    )
    return parser


def _print_status(msg: ExcavationStatus):
    """Print one human-readable excavation status snapshot."""
    state = STATE_LABELS.get(msg.state, f"unknown({msg.state})")
    fault = FAULT_LABELS.get(msg.fault_code, f"unknown({msg.fault_code})")
    print(f"state: {state}")
    print(f"fault: {fault}")
    print(f"homed: {msg.homed}")
    print(f"motor_enabled: {msg.motor_enabled}")
    print(f"estop_active: {msg.estop_active}")
    print(f"driver_fault: {msg.driver_fault}")
    print(f"motor_current_a: {msg.motor_current_a:.2f}")


def main(args=None):
    """Run the excavation bench CLI."""
    parser = _build_parser()
    parsed = parser.parse_args(args=args)

    rclpy.init()
    node = ExcavationBench()
    exit_code = 0

    try:
        if parsed.command == "status":
            _print_status(node._wait_for_status(parsed.timeout))
        elif parsed.command == "home":
            response = node._call_trigger(node._home_client, parsed.timeout)
            print(response.message)
            exit_code = 0 if response.success else 1
        elif parsed.command == "start":
            response = node._call_trigger(node._start_client, parsed.timeout)
            print(response.message)
            exit_code = 0 if response.success else 1
        elif parsed.command == "stop":
            response = node._call_trigger(node._stop_client, parsed.timeout)
            print(response.message)
            exit_code = 0 if response.success else 1
        elif parsed.command == "clear-fault":
            response = node._call_trigger(node._clear_fault_client, parsed.timeout)
            print(response.message)
            exit_code = 0 if response.success else 1
        elif parsed.command == "jog-forward":
            response = node._call_jog(parsed.duration, parsed.timeout)
            print(response.message)
            exit_code = 0 if response.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(exit_code)
