"""Launch file for the localisation stack."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate a launch description for the EKF node.

    This includes the EKF node which fuses odometry and IMU data.
    """
    # Locate the configuration files
    pkg_localisation = get_package_share_directory("lunabot_localisation")
    ekf_config_path = os.path.join(pkg_localisation, "config", "ekf.yaml")

    # Extended Kalman Filter (EKF) Node
    # Fuses multiple odometry sources into a single smooth odom -> base_footprint transform
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            ekf_node,
        ]
    )
