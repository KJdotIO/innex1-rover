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

"""Launch the drivetrain bridge for first-motion bench tests."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _share_path(package: str, *parts: str) -> str:
    """Return a path inside a package share directory."""
    return str(Path(get_package_share_directory(package)).joinpath(*parts))


def _is_truthy(value: str) -> bool:
    """Return whether a launch string means true."""
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _create_optional_teleop_nodes(context):
    """Create joystick teleop and mux nodes only when requested."""
    if not _is_truthy(LaunchConfiguration("enable_teleop").perform(context)):
        return []

    teleop_launch_path = _share_path(
        "lunabot_teleop", "launch", "joystick_teleop.launch.py"
    )
    mux_config_path = _share_path(
        "lunabot_drivetrain", "config", "bench_twist_mux.yaml"
    )

    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_path),
        launch_arguments={
            "joy_device_id": LaunchConfiguration("joy_device_id"),
            "joy_device_name": LaunchConfiguration("joy_device_name"),
            "use_sim_time": "false",
        }.items(),
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        remappings=[("cmd_vel_out", "cmd_vel_safe")],
        parameters=[mux_config_path],
    )

    return [joystick_teleop, twist_mux]


def generate_launch_description():
    """Generate the drivetrain bench launch description."""
    config_path = _share_path("lunabot_drivetrain", "config", "drivetrain.yaml")

    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    serial_protocol = LaunchConfiguration("serial_protocol")
    max_throttle = LaunchConfiguration("max_throttle")
    dry_run = LaunchConfiguration("dry_run")

    drivetrain_bridge = Node(
        package="lunabot_drivetrain",
        executable="drivetrain_bridge",
        name="drivetrain_bridge",
        output="screen",
        parameters=[
            config_path,
            {
                "serial_port": ParameterValue(serial_port, value_type=str),
                "baud_rate": ParameterValue(baud_rate, value_type=int),
                "serial_protocol": ParameterValue(
                    serial_protocol, value_type=str
                ),
                "max_throttle": ParameterValue(max_throttle, value_type=float),
                "dry_run": ParameterValue(dry_run, value_type=bool),
            },
        ],
    )

    velocity_gate = Node(
        package="lunabot_drivetrain",
        executable="velocity_gate",
        name="velocity_gate",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyTHS1",
                description="Jetson UART device connected to the Sabertooth S1 pins.",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="9600",
                description="Sabertooth serial baud rate.",
            ),
            DeclareLaunchArgument(
                "serial_protocol",
                default_value="legacy_simplified",
                description=(
                    "Sabertooth protocol: legacy_simplified or packetized."
                ),
            ),
            DeclareLaunchArgument(
                "max_throttle",
                default_value="0.2",
                description="Temporary throttle cap for first-motion tests.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description=(
                    "Allow launch without a serial controller. Keep false for "
                    "real first-motion tests so missing hardware fails closed."
                ),
            ),
            DeclareLaunchArgument(
                "enable_teleop",
                default_value="false",
                description="Start joystick teleop and route it through the safety gate.",
            ),
            DeclareLaunchArgument(
                "joy_device_id",
                default_value="0",
                description="SDL device index for the connected controller.",
            ),
            DeclareLaunchArgument(
                "joy_device_name",
                default_value="",
                description="Optional SDL controller name to match.",
            ),
            velocity_gate,
            drivetrain_bridge,
            OpaqueFunction(function=_create_optional_teleop_nodes),
        ]
    )
