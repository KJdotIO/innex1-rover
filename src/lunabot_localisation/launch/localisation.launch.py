import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Create a ROS 2 LaunchDescription that starts and configures localization-related nodes for the lunabot_localisation package.
    
    This function resolves the package share directory, loads rtabmap and EKF YAML configuration files, and defines camera topic remappings. It returns a LaunchDescription containing three Node actions:
    - an RGB-D odometry node (rtabmap_odom/rgbd_odometry) configured with rtabmap.yaml, sim time enabled, camera remappings, and an odom remap to /visual_odometry;
    - an EKF filter node (robot_localization/ekf_node named ekf_filter_node) configured with ekf.yaml and sim time enabled;
    - a SLAM node (rtabmap_slam/rtabmap) configured with rtabmap.yaml, sim time enabled, camera remappings, an odom remap to /odometry/filtered, and started with the "-d" argument.
    
    Returns:
        LaunchDescription: A launch description that starts the odometry, EKF, and SLAM nodes with the described configurations.
    """
    pkg_localisation = get_package_share_directory("lunabot_localisation")

    rtabmap_yaml = os.path.join(pkg_localisation, "config", "rtabmap.yaml")
    ekf_yaml = os.path.join(pkg_localisation, "config", "ekf.yaml")

    camera_remappings = [
        ("rgb/image", "/camera_front/image"),
        ("depth/image", "/camera_front/depth_image"),
        ("rgb/camera_info", "/camera_front/camera_info"),
    ]

    return LaunchDescription(
        [
            # vodom
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=camera_remappings + [("odom", "/visual_odometry")],
            ),
            # ekf fuses wheel odom + IMU + visual odom
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True}],
            ),
            # slam node (map -> odom)
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=camera_remappings + [("odom", "/odometry/filtered")],
                arguments=["-d"],
            ),
        ]
    )