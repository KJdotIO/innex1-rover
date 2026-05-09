"""Launch the rover diagnostics aggregator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the rover diagnostics launch description."""
    stale_timeout_s = LaunchConfiguration("stale_timeout_s")
    publish_hz = LaunchConfiguration("publish_hz")

    diagnostics = Node(
        package="lunabot_bringup",
        executable="rover_diagnostics",
        name="rover_diagnostics",
        output="screen",
        parameters=[
            {
                "stale_timeout_s": ParameterValue(
                    stale_timeout_s, value_type=float
                ),
                "publish_hz": ParameterValue(publish_hz, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "stale_timeout_s",
                default_value="2.5",
                description="Seconds before a monitored topic is STALE.",
            ),
            DeclareLaunchArgument(
                "publish_hz",
                default_value="1.0",
                description="DiagnosticArray publish rate.",
            ),
            diagnostics,
        ]
    )
