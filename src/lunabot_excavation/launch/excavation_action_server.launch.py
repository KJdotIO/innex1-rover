"""Launch the excavation action adapter."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the excavation action server node."""
    return LaunchDescription(
        [
            Node(
                package="lunabot_excavation",
                executable="excavation_action_server",
                name="excavation_action_server",
                output="screen",
            )
        ]
    )
