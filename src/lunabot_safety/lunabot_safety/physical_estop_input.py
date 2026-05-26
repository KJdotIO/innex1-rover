"""Publish the physical E-stop input on /safety/estop."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def interpret_estop_input(raw_value: bool, *, active_high: bool) -> bool:
    """Return whether the E-stop is active for one raw input sample."""
    value = bool(raw_value)
    return value if active_high else not value


class PhysicalEstopInput(Node):
    """Publish a debounced physical or configured E-stop input."""

    def __init__(self) -> None:
        super().__init__("physical_estop_input")
        self.declare_parameter("backend", "parameter")
        self.declare_parameter("gpio_pin", 0)
        self.declare_parameter("gpio_numbering", "BOARD")
        self.declare_parameter("active_high", True)
        self.declare_parameter("simulated_estop_active", False)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("release_debounce_samples", 3)
        self.declare_parameter("fail_safe_on_error", True)

        self._backend = str(self.get_parameter("backend").value)
        self._active_high = bool(self.get_parameter("active_high").value)
        self._fail_safe_on_error = bool(
            self.get_parameter("fail_safe_on_error").value
        )
        publish_hz = float(self.get_parameter("publish_hz").value)
        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        self._release_debounce_samples = int(
            self.get_parameter("release_debounce_samples").value
        )
        if self._release_debounce_samples <= 0:
            raise ValueError("release_debounce_samples must be positive")

        self._publisher = self.create_publisher(Bool, "/safety/estop", 10)
        self._gpio = None
        self._gpio_pin = int(self.get_parameter("gpio_pin").value)
        self._published_estop_active: bool | None = None
        self._release_sample_count = 0
        if self._backend == "jetson_gpio":
            self._setup_gpio()
        elif self._backend != "parameter":
            raise ValueError("backend must be 'parameter' or 'jetson_gpio'")

        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            f"physical_estop_input publishing /safety/estop via {self._backend}"
        )

    def _setup_gpio(self) -> None:
        """Initialise Jetson.GPIO for the configured input pin."""
        try:
            import Jetson.GPIO as GPIO  # type: ignore[import-not-found]
        except Exception as exc:
            if self._fail_safe_on_error:
                self.get_logger().error(
                    f"Jetson.GPIO unavailable; publishing E-stop active: {exc}"
                )
                return
            raise

        numbering = str(self.get_parameter("gpio_numbering").value).upper()
        mode = GPIO.BOARD if numbering == "BOARD" else GPIO.BCM
        GPIO.setmode(mode)
        GPIO.setup(self._gpio_pin, GPIO.IN)
        self._gpio = GPIO

    def _read_estop_active(self) -> bool:
        """Read the configured E-stop source."""
        if self._backend == "parameter":
            return bool(self.get_parameter("simulated_estop_active").value)

        if self._gpio is None:
            return self._fail_safe_on_error

        raw_value = bool(self._gpio.input(self._gpio_pin))
        return interpret_estop_input(raw_value, active_high=self._active_high)

    def _debounced_estop_active(self, sampled_active: bool) -> bool:
        """Apply immediate assertion and debounced release."""
        if self._published_estop_active is None:
            self._published_estop_active = sampled_active
            self._release_sample_count = 0
            return sampled_active

        if sampled_active:
            self._published_estop_active = True
            self._release_sample_count = 0
            return True

        if not self._published_estop_active:
            self._release_sample_count = 0
            return False

        self._release_sample_count += 1
        if self._release_sample_count >= self._release_debounce_samples:
            self._published_estop_active = False
            self._release_sample_count = 0

        return self._published_estop_active

    def _publish(self) -> None:
        msg = Bool()
        msg.data = self._debounced_estop_active(self._read_estop_active())
        self._publisher.publish(msg)

    def destroy_node(self):
        """Clean up GPIO resources before shutdown."""
        if self._gpio is not None:
            self._gpio.cleanup(self._gpio_pin)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicalEstopInput()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
