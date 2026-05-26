"""Launch the Ouster OS1 debug stream with a Foxglove bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate the Ouster LiDAR debug launch description."""
    bringup_share = Path(get_package_share_directory("lunabot_bringup"))
    ouster_share = Path(get_package_share_directory("ouster_ros"))

    params_file = LaunchConfiguration("params_file")
    foxglove_port = LaunchConfiguration("foxglove_port")
    send_buffer_limit = LaunchConfiguration("send_buffer_limit")
    default_params_file = bringup_share / "config" / "ouster_lidar_debug.yaml"

    ouster_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ouster_share / "launch" / "driver.launch.py")
        ),
        launch_arguments={
            "params_file": params_file,
            "viz": "False",
            "ouster_ns": "ouster",
            "os_driver_name": "os_driver",
        }.items(),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="ouster_foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": ParameterValue(foxglove_port, value_type=int),
                "send_buffer_limit": ParameterValue(
                    send_buffer_limit,
                    value_type=int,
                ),
                "topic_whitelist": [
                    "/ouster/points",
                    "/ouster/imu",
                    "/ouster/scan",
                    "/ouster/metadata",
                    "/ouster/telemetry",
                    "/tf",
                    "/tf_static",
                ],
                "capabilities": ["clientPublish"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params_file),
                description="Ouster ROS driver parameter file.",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="Foxglove WebSocket port for the Ouster debug view.",
            ),
            DeclareLaunchArgument(
                "send_buffer_limit",
                default_value="100000000",
                description=(
                    "Foxglove per-client send buffer in bytes for point cloud debug."
                ),
            ),
            ouster_driver,
            foxglove_bridge,
        ]
    )
