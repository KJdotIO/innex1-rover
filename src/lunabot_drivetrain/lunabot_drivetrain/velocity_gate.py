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

"""Zero cmd_vel when the drivetrain is faulted or motion is inhibited."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool

from lunabot_interfaces.msg import DrivetrainStatus

_INHIBIT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_ZERO_TWIST = Twist()


class VelocityGate(Node):
    """Pass or zero cmd_vel based on drivetrain health.

    Sits between the collision monitor output (/cmd_vel_safe) and the
    drivetrain bridge input (/cmd_vel_gated).  Provides a second
    layer of safety beyond the bridge's own inhibit checks.
    """

    _ALLOWED_STATES = frozenset({
        DrivetrainStatus.STATE_READY,
        DrivetrainStatus.STATE_DRIVING,
    })

    def __init__(self) -> None:
        super().__init__("velocity_gate")
        self._gate_open = False
        self._motion_inhibited = False

        self._cmd_sub = self.create_subscription(
            Twist, "/cmd_vel_safe", self._cmd_callback, 10
        )
        self._status_sub = self.create_subscription(
            DrivetrainStatus,
            "/drivetrain/status",
            self._status_callback,
            10,
        )
        self._inhibit_sub = self.create_subscription(
            Bool,
            "/safety/motion_inhibit",
            self._inhibit_callback,
            _INHIBIT_QOS,
        )
        self._cmd_pub = self.create_publisher(
            Twist, "/cmd_vel_gated", 10
        )

        self.get_logger().info("Velocity gate started (gate closed)")

    def _status_callback(self, msg: DrivetrainStatus) -> None:
        """Update gate based on drivetrain state."""
        was_open = self._gate_open
        self._gate_open = (
            msg.state in self._ALLOWED_STATES
            and not msg.estop_active
            and not msg.motion_inhibited
        )
        if was_open and not self._gate_open:
            self.get_logger().warn("Gate CLOSED — drivetrain unhealthy")
            self._cmd_pub.publish(_ZERO_TWIST)
        elif not was_open and self._gate_open:
            self.get_logger().info("Gate OPEN — drivetrain healthy")

    def _inhibit_callback(self, msg: Bool) -> None:
        """Track motion inhibit independently."""
        self._motion_inhibited = msg.data
        if msg.data:
            self._gate_open = False
            self._cmd_pub.publish(_ZERO_TWIST)

    def _cmd_callback(self, msg: Twist) -> None:
        """Forward or zero the velocity command."""
        if self._gate_open and not self._motion_inhibited:
            self._cmd_pub.publish(msg)
        else:
            self._cmd_pub.publish(_ZERO_TWIST)


def main(args=None) -> None:
    """Entry point for the velocity_gate executable."""
    rclpy.init(args=args)
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
