from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a bench-ready material launch with explicit excavation mock telemetry."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fault_on_start_code",
                default_value="0",
                description="Bench mock fault code to latch after a start command",
            ),
            DeclareLaunchArgument(
                "use_mock_telemetry",
                default_value="true",
                description="Launch the explicit excavation telemetry mock for bench-only use",
            ),
            DeclareLaunchArgument(
                "fault_on_stop_code",
                default_value="0",
                description="Bench mock fault code to latch after a stop command",
            ),
            Node(
                package="lunabot_excavation",
                executable="excavation_telemetry_mock",
                name="excavation_telemetry_mock",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_mock_telemetry")),
                parameters=[
                    {
                        "fault_on_start_code": LaunchConfiguration("fault_on_start_code"),
                        "fault_on_stop_code": LaunchConfiguration("fault_on_stop_code"),
                    }
                ],
            ),
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
