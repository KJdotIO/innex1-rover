import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_localisation = get_package_share_directory("lunabot_localisation")

    rtabmap_yaml = os.path.join(pkg_localisation, "config", "rtabmap.yaml")
    ekf_yaml = os.path.join(pkg_localisation, "config", "ekf.yaml")
    apriltag_yaml = os.path.join(pkg_localisation, "config", "apriltag.yaml")

    camera_remappings = [
        ("rgb/image", "/camera_front/image"),
        ("depth/image", "/camera_front/depth_image"),
        ("rgb/camera_info", "/camera_front/camera_info"),
    ]

    return LaunchDescription(
        [
            # Visual odometry
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[*camera_remappings, ("odom", "/visual_odometry")],
            ),
            # Local EKF: odom -> base_footprint (smooth, continuous)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True}],
            ),
            # Global EKF: map -> odom (corrects drift when tag seen)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "/odometry/global")],
            ),
            # RTAB-Map SLAM (map building only, no TF publishing)
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[*camera_remappings, ("odom", "/odometry/filtered")],
                arguments=["-d"],
            ),
            # AprilTag detector
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
            # Known position of tag 0 in the map frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0",
                    "--y", "1.08",
                    "--z", "0.25",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "tag36h11:0",
                ],
            ),
            # Converts apriltag TF into PoseWithCovarianceStamped for global EKF
            Node(
                package="lunabot_localisation",
                executable="tag_pose_publisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
