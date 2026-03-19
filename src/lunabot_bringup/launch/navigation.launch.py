"""Launch file for the navigation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Generate a launch description for the navigation stack.

    This includes the EKF localisation and the Nav2 servers.
    """
    # Locate the configuration files
    pkg_bringup = get_package_share_directory("lunabot_bringup")
    pkg_nav = get_package_share_directory("lunabot_navigation")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    nav_params_path = os.path.join(pkg_nav, "config", "nav2_params.yaml")

    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "localisation.launch.py")
        ),
        launch_arguments={"lidar_costmap_phase": "true"}.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav_params_path,
            "autostart": "true",
        }.items(),
    )

    # Delay Nav2 startup slightly so sim time / TF / sensor streams can settle.
    delayed_nav2_launch = TimerAction(period=5.0, actions=[nav2_launch])

    return LaunchDescription([localisation_launch, delayed_nav2_launch])
