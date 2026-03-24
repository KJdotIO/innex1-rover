"""Launch excavation controller nodes and the deposition stub."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate a launch description for excavation and deposition action paths.

    Excavation uses the real controller and action adapter. Deposition remains stubbed.
    """
    return LaunchDescription(
        [
            Node(
                package="lunabot_excavation",
                executable="excavation_controller",
                name="excavation_controller",
                output="screen",
            ),
            Node(
                package="lunabot_excavation",
                executable="excavation_action_server",
                name="excavation_action_server",
                output="screen",
            ),
            Node(
                package="lunabot_control",
                executable="material_action_server",
                name="material_action_server",
                output="screen",
            )
        ]
    )
