"""Launch file for the localisation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Generate a launch description for the localisation stack.

    Includes the master localisation launch from lunabot_localisation.
    """
    pkg_localisation = get_package_share_directory("lunabot_localisation")
    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localisation, "launch", "localisation.launch.py")
        )
    )
    return LaunchDescription([localisation_launch])
