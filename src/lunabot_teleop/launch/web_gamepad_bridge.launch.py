"""Launch the browser-based gamepad bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for browser-to-ROS teleop."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bind_host",
                default_value="0.0.0.0",
                description="HTTP bind address for the browser controller page.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8080",
                description="HTTP port for the browser controller page.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel_safe",
                description="ROS Twist topic published by the web bridge.",
            ),
            DeclareLaunchArgument(
                "tls_cert_file",
                default_value="",
                description="Optional TLS certificate file for HTTPS.",
            ),
            DeclareLaunchArgument(
                "tls_key_file",
                default_value="",
                description="Optional TLS key file for HTTPS.",
            ),
            DeclareLaunchArgument(
                "max_linear_mps",
                default_value="0.30",
                description="Maximum browser-commanded linear speed.",
            ),
            DeclareLaunchArgument(
                "max_angular_radps",
                default_value="0.80",
                description="Maximum browser-commanded angular speed.",
            ),
            DeclareLaunchArgument(
                "command_timeout_s",
                default_value="0.35",
                description="Publish zero if browser commands stop for this long.",
            ),
            Node(
                package="lunabot_teleop",
                executable="web_gamepad_bridge",
                name="web_gamepad_bridge",
                output="screen",
                parameters=[
                    {
                        "bind_host": LaunchConfiguration("bind_host"),
                        "port": ParameterValue(
                            LaunchConfiguration("port"),
                            value_type=int,
                        ),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "tls_cert_file": LaunchConfiguration("tls_cert_file"),
                        "tls_key_file": LaunchConfiguration("tls_key_file"),
                        "max_linear_mps": ParameterValue(
                            LaunchConfiguration("max_linear_mps"),
                            value_type=float,
                        ),
                        "max_angular_radps": ParameterValue(
                            LaunchConfiguration("max_angular_radps"),
                            value_type=float,
                        ),
                        "command_timeout_s": ParameterValue(
                            LaunchConfiguration("command_timeout_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
