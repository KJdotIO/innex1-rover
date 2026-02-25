"""Launch file for the navigation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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
        )
    )

    hazard_detection_node = Node(
        package="lunabot_perception",
        executable="hazard_detection",
        name="hazard_detection",
        output="screen",
        parameters=[{"use_sim_time": True}],
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

    return LaunchDescription([localisation_launch, hazard_detection_node, nav2_launch])
