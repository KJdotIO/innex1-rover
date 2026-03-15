"""
Launch the complete simulation stack: Gazebo + localisation + Nav2.

Single command to start everything needed for sim testing:
  ros2 launch lunabot_bringup simulation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate a launch description for the full simulation stack."""
    pkg_bringup = get_package_share_directory("lunabot_bringup")
    pkg_simulation = get_package_share_directory("lunabot_simulation")

    # 1. Gazebo simulation (server + bridges)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, "launch", "moon_yard.launch.py")
        )
    )

    # 2. Localisation + Nav2 (delayed 5s to let Gazebo spawn the robot first)
    nav_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "navigation.launch.py")
                )
            ),
        ],
    )

    return LaunchDescription([gazebo_launch, nav_launch])
