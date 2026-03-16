import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localisation = get_package_share_directory("lunabot_localisation")

    ekf_yaml = os.path.join(pkg_localisation, "config", "ekf.yaml")
    apriltag_yaml = os.path.join(pkg_localisation, "config", "apriltag.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "/odometry/global")],
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                output="screen",
                parameters=[apriltag_yaml, {"use_sim_time": True}],
                remappings=[
                    ("image_rect", "/camera_front/image"),
                    ("camera_info", "/camera_front/camera_info"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "-2.95",
                    "--y",
                    "2.15",
                    "--z",
                    "0.12",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "tag36h11:0",
                ],
            ),
            Node(
                package="lunabot_localisation",
                executable="tag_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "camera_frame": "camera_front_optical_frame",
                    }
                ],
            ),
        ]
    )
