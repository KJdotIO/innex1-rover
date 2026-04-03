"""Launch file for the localisation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate a launch description for the localisation stack.

    Includes the master localisation launch from lunabot_localisation.
    """
    pkg_localisation = get_package_share_directory("lunabot_localisation")
    lidar_costmap_phase = LaunchConfiguration("lidar_costmap_phase")
    enable_visual_slam = LaunchConfiguration("enable_visual_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_apriltag_debug = LaunchConfiguration("enable_apriltag_debug")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localisation, "launch", "localisation.launch.py")
        ),
        launch_arguments={
            "lidar_costmap_phase": lidar_costmap_phase,
            "enable_visual_slam": enable_visual_slam,
            "use_sim_time": use_sim_time,
            "enable_apriltag_debug": enable_apriltag_debug,
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_costmap_phase",
                default_value="false",
                description="Use odom-only debug localisation with an identity map->odom TF.",
            ),
            DeclareLaunchArgument(
                "enable_visual_slam",
                default_value="false",
                description=(
                    "Optionally enable RTAB-Map odometry and mapping for "
                    "experimentation; the June EKF baseline does not fuse VO."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock instead of wall time for all launched nodes.",
            ),
            DeclareLaunchArgument(
                "enable_apriltag_debug",
                default_value="false",
                description=(
                    "Launch the apriltag_draw overlay for annotated front camera "
                    "debugging."
                ),
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="cmd_vel",
                description="Velocity command topic used by the start-zone localiser.",
            ),
            localisation_launch,
        ]
    )
