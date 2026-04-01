"""Launch the hardware localisation stack with the OAK-D Pro wrapper."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the hardware localisation launch description."""
    pkg_bringup = get_package_share_directory("lunabot_bringup")
    pkg_sensors = get_package_share_directory("lunabot_sensors")

    profile = LaunchConfiguration("profile")
    enable_apriltag_debug = LaunchConfiguration("enable_apriltag_debug")
    lidar_costmap_phase = LaunchConfiguration("lidar_costmap_phase")
    enable_visual_slam = LaunchConfiguration("enable_visual_slam")

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensors, "launch", "oakd_pro.launch.py")
        ),
        launch_arguments={
            "profile": profile,
            "use_sim_time": "false",
        }.items(),
    )

    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "localisation.launch.py")
        ),
        launch_arguments={
            "lidar_costmap_phase": lidar_costmap_phase,
            "enable_visual_slam": enable_visual_slam,
            "use_sim_time": "false",
            "enable_apriltag_debug": enable_apriltag_debug,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="usb2_degraded",
                description="DepthAI hardware profile to launch.",
            ),
            DeclareLaunchArgument(
                "lidar_costmap_phase",
                default_value="false",
                description="Keep the existing localisation debug mode available.",
            ),
            DeclareLaunchArgument(
                "enable_visual_slam",
                default_value="false",
                description="Leave RTAB-Map off by default on hardware.",
            ),
            DeclareLaunchArgument(
                "enable_apriltag_debug",
                default_value="false",
                description="Optionally launch apriltag_draw on hardware.",
            ),
            camera_launch,
            localisation_launch,
        ]
    )
