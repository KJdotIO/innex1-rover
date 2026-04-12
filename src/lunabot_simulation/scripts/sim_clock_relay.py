#!/usr/bin/env python3

"""Republish Gazebo clock updates without allowing time to move backwards."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


@dataclass(frozen=True)
class _ClockStamp:
    sec: int
    nanosec: int

    @classmethod
    def from_msg(cls, msg: Clock) -> "_ClockStamp":
        return cls(sec=msg.clock.sec, nanosec=msg.clock.nanosec)

    def to_msg(self) -> Clock:
        msg = Clock()
        msg.clock.sec = self.sec
        msg.clock.nanosec = self.nanosec
        return msg

    def is_before(self, other: "_ClockStamp") -> bool:
        return (self.sec, self.nanosec) < (other.sec, other.nanosec)


class SimClockRelay(Node):
    """Clamp backwards sim-time jumps so the ROS graph sees a monotonic clock."""

    def __init__(self) -> None:
        super().__init__("sim_clock_relay")
        self._latest_clock: _ClockStamp | None = None
        self._clamped_messages = 0
        self._clock_publisher = self.create_publisher(Clock, "/clock", 10)
        self.create_subscription(Clock, "/clock/raw", self._on_clock, 10)

    def _on_clock(self, msg: Clock) -> None:
        incoming_clock = _ClockStamp.from_msg(msg)
        outgoing_clock = incoming_clock
        if self._latest_clock and incoming_clock.is_before(self._latest_clock):
            self._clamped_messages += 1
            outgoing_clock = self._latest_clock
            if self._clamped_messages <= 5 or self._clamped_messages % 50 == 0:
                self.get_logger().warning(
                    "Clamped backwards sim-time jump from "
                    f"{incoming_clock.sec}.{incoming_clock.nanosec:09d} to "
                    f"{self._latest_clock.sec}.{self._latest_clock.nanosec:09d}",
                )

        self._latest_clock = outgoing_clock
        self._clock_publisher.publish(outgoing_clock.to_msg())


def main() -> None:
    rclpy.init()
    node = SimClockRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
