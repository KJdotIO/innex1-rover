"""Launch material handling action stubs for excavation and deposition."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate a launch description for material action stubs.

    Launches the material action server used for excavation/deposition bench tests.
    """
    return LaunchDescription(
        [
            Node(
                package="lunabot_control",
                executable="material_action_server",
                name="material_action_server",
                output="screen",
            )
        ]
    )
