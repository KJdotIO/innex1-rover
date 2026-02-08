"""Launch file for the localisation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Includes the master localisation launch from the lunabot_localisation package.
    
    This assembles the three-node localisation stack:
    - Visual odometry (`rgbd_odometry`)
    - EKF fusion (`robot_localization`)
    - SLAM (`rtabmap`)
    
    Returns:
        LaunchDescription: A launch description that includes the master localisation launch.
    """
    pkg_localisation = get_package_share_directory("lunabot_localisation")
    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localisation, "launch", "localisation.launch.py")
        )
    )
    return LaunchDescription([localisation_launch])