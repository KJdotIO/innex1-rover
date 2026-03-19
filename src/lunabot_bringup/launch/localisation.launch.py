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
    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localisation, "launch", "localisation.launch.py")
        ),
        launch_arguments={"lidar_costmap_phase": lidar_costmap_phase}.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_costmap_phase",
                default_value="true",
                description="Launch minimal odom-only localisation for lidar Nav2 bringup.",
            ),
            localisation_launch,
        ]
    )
