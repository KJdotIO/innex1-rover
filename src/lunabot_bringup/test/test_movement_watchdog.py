"""Unit tests for the advisory movement watchdog."""

import importlib
import math
import sys
import types


def _install_ros_stubs():
    diagnostic_msgs = types.ModuleType("diagnostic_msgs")
    diagnostic_msgs_msg = types.ModuleType("diagnostic_msgs.msg")

    class DiagnosticStatus:
        OK = 0
        WARN = 1
        ERROR = 2
        STALE = 3

        def __init__(self):
            self.name = ""
            self.hardware_id = ""
            self.level = self.OK
            self.message = ""
            self.values = []

    class DiagnosticArray:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)
            self.status = []

    class KeyValue:
        def __init__(self):
            self.key = ""
            self.value = ""

    diagnostic_msgs_msg.DiagnosticArray = DiagnosticArray
    diagnostic_msgs_msg.DiagnosticStatus = DiagnosticStatus
    diagnostic_msgs_msg.KeyValue = KeyValue

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")

    class Odometry:
        def __init__(self):
            orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            pose = types.SimpleNamespace(position=position, orientation=orientation)
            self.pose = types.SimpleNamespace(pose=pose)

    nav_msgs_msg.Odometry = Odometry

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node.Node = Node

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")

    class SetBool:
        class Request:
            data = False

        class Response:
            success = False
            message = ""

    std_srvs_srv.SetBool = SetBool

    lunabot_interfaces = types.ModuleType("lunabot_interfaces")
    lunabot_interfaces_msg = types.ModuleType("lunabot_interfaces.msg")

    class DrivetrainStatus:
        STATE_DRIVING = 2
        FAULT_NONE = 0

        def __init__(self):
            self.state = 0
            self.fault_code = self.FAULT_NONE

    class DrivetrainTelemetry:
        def __init__(self):
            self.wheel_velocity_rps = [0.0, 0.0, 0.0, 0.0]
            self.encoder_ticks = [0, 0, 0, 0]

    lunabot_interfaces_msg.DrivetrainStatus = DrivetrainStatus
    lunabot_interfaces_msg.DrivetrainTelemetry = DrivetrainTelemetry
    lunabot_interfaces.msg = lunabot_interfaces_msg

    sys.modules.setdefault("diagnostic_msgs", diagnostic_msgs)
    sys.modules.setdefault("diagnostic_msgs.msg", diagnostic_msgs_msg)
    sys.modules.setdefault("nav_msgs", nav_msgs)
    sys.modules.setdefault("nav_msgs.msg", nav_msgs_msg)
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy_node)
    sys.modules.setdefault("std_srvs", std_srvs)
    sys.modules.setdefault("std_srvs.srv", std_srvs_srv)
    sys.modules.setdefault("lunabot_interfaces", lunabot_interfaces)
    sys.modules.setdefault("lunabot_interfaces.msg", lunabot_interfaces_msg)


try:
    from nav_msgs.msg import Odometry
except ModuleNotFoundError:
    _install_ros_stubs()
    from nav_msgs.msg import Odometry

movement_watchdog = importlib.import_module("lunabot_bringup.movement_watchdog")
lunabot_interface_msgs = importlib.import_module("lunabot_interfaces.msg")

LEVEL_ERROR = movement_watchdog.LEVEL_ERROR
LEVEL_OK = movement_watchdog.LEVEL_OK
LEVEL_STALE = movement_watchdog.LEVEL_STALE
LEVEL_WARN = movement_watchdog.LEVEL_WARN
MovementWatchdog = movement_watchdog.MovementWatchdog
MovementWatchdogConfig = movement_watchdog.MovementWatchdogConfig
DrivetrainStatus = lunabot_interface_msgs.DrivetrainStatus
DrivetrainTelemetry = lunabot_interface_msgs.DrivetrainTelemetry


def _watchdog(**config_overrides):
    config = MovementWatchdogConfig(
        timeout_s=10.0,
        warn_s=8.0,
        stale_timeout_s=2.0,
        confirmation_window_s=1.0,
        min_translation_m=0.10,
        min_yaw_rad=0.10,
        min_wheel_velocity_rps=0.20,
        min_encoder_ticks=5,
    )
    for key, value in config_overrides.items():
        setattr(config, key, value)
    return MovementWatchdog(config=config)


def _odom(x=0.0, y=0.0, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def _values(status):
    return {item.key: item.value for item in status.values}


def test_disarmed_watchdog_is_ok_without_sources():
    watchdog = _watchdog()

    status = watchdog.diagnostic(now_s=100.0)

    assert status.level == LEVEL_OK
    assert status.message == "Movement watchdog disarmed"
    assert _values(status)["armed"] == "False"


def test_armed_watchdog_is_stale_without_physical_evidence():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)

    status = watchdog.diagnostic(now_s=1.0)

    assert status.level == LEVEL_STALE
    assert "No fresh physical movement evidence" in status.message


def test_fresh_odom_without_movement_warns_then_errors():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(), now_s=0.5)
    watchdog.update_odom("/odometry/local", _odom(), now_s=7.9)

    warning = watchdog.diagnostic(now_s=8.0)
    watchdog.update_odom("/odometry/local", _odom(), now_s=9.9)
    error = watchdog.diagnostic(now_s=10.0)

    assert warning.level == LEVEL_WARN
    assert error.level == LEVEL_ERROR
    assert _values(error)["ever_moved"] == "False"


def test_sustained_odom_translation_confirms_movement_before_timeout():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(x=0.00), now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(x=0.06), now_s=0.5)
    watchdog.update_odom("/odometry/local", _odom(x=0.12), now_s=1.1)
    watchdog.update_odom("/odometry/local", _odom(x=0.12), now_s=6.9)

    status = watchdog.diagnostic(now_s=7.0)

    assert status.level == LEVEL_OK
    assert watchdog.ever_moved is True
    assert _values(status)["last_motion_source"] == "/odometry/local"
    assert _values(status)["seconds_since_confirmed_motion"] == "5.90"


def test_pre_arm_motion_is_not_counted_after_arming():
    watchdog = _watchdog()
    watchdog.update_odom("/odometry/local", _odom(x=0.00), now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(x=0.20), now_s=2.0)

    watchdog.set_armed(True, now_s=3.0)
    watchdog.update_odom("/odometry/local", _odom(x=0.20), now_s=3.1)
    status = watchdog.diagnostic(now_s=3.2)

    assert watchdog.ever_moved is False
    assert status.level == LEVEL_OK
    assert status.message == "Awaiting confirmed rover movement"


def test_sustained_rotation_counts_as_movement():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(yaw=0.00), now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(yaw=0.06), now_s=0.5)
    watchdog.update_odom("/odometry/local", _odom(yaw=0.12), now_s=1.1)

    status = watchdog.diagnostic(now_s=2.0)

    assert status.level == LEVEL_OK
    assert watchdog.ever_moved is True


def test_intermittent_twitch_does_not_confirm_movement():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(x=0.00), now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(x=0.12), now_s=0.1)
    watchdog.update_odom("/odometry/local", _odom(x=0.12), now_s=0.2)
    watchdog.update_odom("/odometry/local", _odom(x=0.12), now_s=7.9)

    status = watchdog.diagnostic(now_s=8.0)

    assert watchdog.ever_moved is False
    assert status.level == LEVEL_WARN


def test_drivetrain_state_driving_does_not_count_as_movement_evidence():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)
    msg = DrivetrainStatus()
    msg.state = DrivetrainStatus.STATE_DRIVING
    msg.fault_code = DrivetrainStatus.FAULT_NONE

    watchdog.update_drivetrain_status(msg)
    status = watchdog.diagnostic(now_s=10.0)

    assert status.level == LEVEL_STALE
    assert watchdog.ever_moved is False
    assert _values(status)["drivetrain_state"] == str(DrivetrainStatus.STATE_DRIVING)


def test_prefers_local_odometry_over_fresh_fallback_source():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)
    watchdog.update_odom("/odom", _odom(x=0.0), now_s=0.1)
    watchdog.update_odom("/odometry/local", _odom(x=0.0), now_s=0.2)

    status = watchdog.diagnostic(now_s=1.0)

    assert _values(status)["selected_source"] == "/odometry/local"


def test_falls_back_to_odom_when_local_odometry_is_stale():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)
    watchdog.update_odom("/odometry/local", _odom(x=0.0), now_s=0.1)
    watchdog.update_odom("/odom", _odom(x=0.0), now_s=3.0)

    status = watchdog.diagnostic(now_s=3.5)

    assert status.level == LEVEL_OK
    assert _values(status)["selected_source"] == "/odom"


def test_sustained_drivetrain_telemetry_is_last_resort_movement_evidence():
    watchdog = _watchdog()
    watchdog.set_armed(True, now_s=0.0)
    first = DrivetrainTelemetry()
    first.wheel_velocity_rps = [0.3, 0.3, 0.3, 0.3]
    first.encoder_ticks = [0, 0, 0, 0]
    second = DrivetrainTelemetry()
    second.wheel_velocity_rps = [0.3, 0.3, 0.3, 0.3]
    second.encoder_ticks = [2, 2, 2, 2]

    watchdog.update_telemetry(first, now_s=0.0)
    watchdog.update_telemetry(second, now_s=1.1)
    status = watchdog.diagnostic(now_s=2.0)

    assert status.level == LEVEL_OK
    assert watchdog.ever_moved is True
    assert _values(status)["last_motion_source"] == "/drivetrain/telemetry"
