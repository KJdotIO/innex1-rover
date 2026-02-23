"""Launch material handling action stubs for excavation and deposition."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Return launch description with one material action server node."""
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
