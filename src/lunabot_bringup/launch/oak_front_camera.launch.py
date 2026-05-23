"""Launch the front OAK-D camera through the DepthAI ROS driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _relay_node(name: str, input_topic: str, output_topic: str) -> Node:
    return Node(
        package="topic_tools",
        executable="relay",
        name=name,
        output="screen",
        parameters=[
            {
                "input_topic": input_topic,
                "output_topic": output_topic,
            }
        ],
    )


def generate_launch_description() -> LaunchDescription:
    """Generate the front OAK-D launch description."""
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_model",
                default_value="OAK-D-PRO",
                description="DepthAI camera model passed to the upstream driver.",
            ),
            DeclareLaunchArgument(
                "enable_pointcloud",
                default_value="true",
                description="Start the upstream RGBD point cloud path.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Open the upstream DepthAI RViz profile.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("depthai_ros_driver"),
                            "launch",
                            "camera.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "name": "camera_front",
                    "camera_model": LaunchConfiguration("camera_model"),
                    "params_file": PathJoinSubstitution(
                        [
                            FindPackageShare("lunabot_bringup"),
                            "config",
                            "oak_front_rgbd.yaml",
                        ]
                    ),
                    "pointcloud.enable": enable_pointcloud,
                    "use_rviz": "false",
                    "rectify_rgb": "true",
                }.items(),
            ),
            _relay_node(
                "camera_front_image_relay",
                "/camera_front/rgb/image_raw",
                "/camera_front/image",
            ),
            _relay_node(
                "camera_front_depth_image_relay",
                "/camera_front/stereo/image_raw",
                "/camera_front/depth_image",
            ),
            _relay_node(
                "camera_front_camera_info_relay",
                "/camera_front/rgb/camera_info",
                "/camera_front/camera_info",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="depth_camera_debug_rviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("lunabot_bringup"),
                            "rviz",
                            "depth_camera_debug.rviz",
                        ]
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
