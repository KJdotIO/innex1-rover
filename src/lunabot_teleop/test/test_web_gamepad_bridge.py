"""Unit tests for the browser gamepad bridge helpers."""

import sys
import types


def _install_ros_fakes():
    """Install small ROS module fakes for helper-only tests on non-ROS hosts."""
    try:
        import rclpy as _rclpy  # noqa: F401
        from geometry_msgs.msg import Twist as _Twist  # noqa: F401

        return
    except ModuleNotFoundError:
        pass

    class _Vector:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class Twist:
        def __init__(self):
            self.linear = _Vector()
            self.angular = _Vector()

    class Node:
        pass

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msg.Twist = Twist
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msg

    class DrivetrainStatus:
        pass

    class DrivetrainTelemetry:
        pass

    lunabot_interfaces = types.ModuleType("lunabot_interfaces")
    lunabot_msg = types.ModuleType("lunabot_interfaces.msg")
    lunabot_msg.DrivetrainStatus = DrivetrainStatus
    lunabot_msg.DrivetrainTelemetry = DrivetrainTelemetry
    sys.modules["lunabot_interfaces"] = lunabot_interfaces
    sys.modules["lunabot_interfaces.msg"] = lunabot_msg

    ament_index_python = types.ModuleType("ament_index_python")
    packages = types.ModuleType("ament_index_python.packages")
    packages.get_package_share_directory = lambda _name: "."
    sys.modules["ament_index_python"] = ament_index_python
    sys.modules["ament_index_python.packages"] = packages

    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.node.Node = Node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy.node


_install_ros_fakes()

from lunabot_teleop.web_gamepad_bridge import (  # noqa: E402
    GamepadCommand,
    SpeedLimits,
    command_to_twist,
    parse_command,
    parse_speed_limits,
)


def test_parse_command_clamps_untrusted_values():
    command = parse_command(
        {"linear_x": 4.0, "angular_z": -2.0, "enabled": True}
    )

    assert command == GamepadCommand(1.0, -1.0, True)


def test_disabled_command_outputs_zero_twist():
    twist = command_to_twist(
        GamepadCommand(1.0, -1.0, False),
        max_linear_mps=0.12,
        max_angular_radps=0.35,
    )

    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_enabled_command_scales_to_limits():
    twist = command_to_twist(
        GamepadCommand(0.5, -0.25, True),
        max_linear_mps=0.12,
        max_angular_radps=0.4,
    )

    assert twist.linear.x == 0.06
    assert twist.angular.z == -0.1


def test_parse_speed_limits_clamps_dashboard_values():
    limits = parse_speed_limits(
        {"max_linear_mps": 4.0, "max_angular_radps": -1.0},
        current_linear_mps=0.3,
        current_angular_radps=0.8,
    )

    assert limits == SpeedLimits(max_linear_mps=0.60, max_angular_radps=0.05)


def test_parse_speed_limits_keeps_current_values_when_omitted():
    limits = parse_speed_limits(
        {},
        current_linear_mps=0.3,
        current_angular_radps=0.8,
    )

    assert limits == SpeedLimits(max_linear_mps=0.3, max_angular_radps=0.8)
