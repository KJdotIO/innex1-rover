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
    movement_timeout_s = LaunchConfiguration("movement_timeout_s")
    movement_warn_s = LaunchConfiguration("movement_warn_s")
    movement_auto_arm = LaunchConfiguration("movement_auto_arm")

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
    movement_watchdog = Node(
        package="lunabot_bringup",
        executable="movement_watchdog",
        name="movement_watchdog",
        output="screen",
        parameters=[
            {
                "timeout_s": ParameterValue(
                    movement_timeout_s, value_type=float
                ),
                "warn_s": ParameterValue(movement_warn_s, value_type=float),
                "stale_timeout_s": ParameterValue(
                    stale_timeout_s, value_type=float
                ),
                "publish_hz": ParameterValue(publish_hz, value_type=float),
                "auto_arm": ParameterValue(
                    movement_auto_arm, value_type=bool
                ),
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
            DeclareLaunchArgument(
                "movement_timeout_s",
                default_value="300.0",
                description="Seconds without confirmed movement before ERROR.",
            ),
            DeclareLaunchArgument(
                "movement_warn_s",
                default_value="240.0",
                description="Seconds without confirmed movement before WARN.",
            ),
            DeclareLaunchArgument(
                "movement_auto_arm",
                default_value="false",
                description="Arm the movement watchdog at launch.",
            ),
            diagnostics,
            movement_watchdog,
        ]
    )
