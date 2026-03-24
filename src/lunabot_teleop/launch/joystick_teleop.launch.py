"""Launch joystick teleoperation nodes for Lunabot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Generate a launch description for joystick teleoperation.

    Start the SDL-backed game controller node and teleop_twist_joy with the
    Lunabot Xbox-style mapping. Teleop commands are published on
    /cmd_vel_teleop so a mux can arbitrate between manual and autonomous input.
    """
    pkg_share = get_package_share_directory("lunabot_teleop")
    teleop_config = os.path.join(pkg_share, "config", "xbox_teleop.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    joy_device_id = LaunchConfiguration("joy_device_id")

    game_controller = Node(
        package="joy",
        executable="game_controller_node",
        name="game_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"device_id": ParameterValue(joy_device_id, value_type=int)},
            {"deadzone": 0.08},
            {"autorepeat_rate": 20.0},
            {"coalesce_interval_ms": 1},
        ],
    )

    teleop_twist = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[teleop_config, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "cmd_vel_teleop")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description=(
                    "Use /clock instead of wall time for teleop nodes."
                ),
            ),
            DeclareLaunchArgument(
                "joy_device_id",
                default_value="0",
                description=(
                    "SDL device index for the connected controller."
                ),
            ),
            game_controller,
            teleop_twist,
        ]
    )
