"""Launch the RViz profile used for OAK-D depth camera bring-up."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate the depth camera debug launch description."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock. Keep false for live OAK-D hardware tests.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Open RViz with the depth camera debug profile.",
            ),
            DeclareLaunchArgument(
                "run_crater_detection",
                default_value="false",
                description=(
                    "Run crater_detection against /camera_front/points. "
                    "Requires TF from the point cloud frame to odom."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("lunabot_bringup"),
                        "rviz",
                        "depth_camera_debug.rviz",
                    ]
                ),
                description="RViz config to load.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="depth_camera_debug_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            ),
            Node(
                package="lunabot_perception",
                executable="crater_detection",
                name="crater_detection_node",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
                condition=IfCondition(LaunchConfiguration("run_crater_detection")),
            ),
        ]
    )
