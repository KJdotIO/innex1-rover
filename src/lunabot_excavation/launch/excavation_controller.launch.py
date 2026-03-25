"""Launch the excavation controller skeleton."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for the excavation controller node."""
    return LaunchDescription(
        [
            Node(
                package="lunabot_excavation",
                executable="excavation_controller",
                name="excavation_controller",
                output="screen",
            )
        ]
    )
