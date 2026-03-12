import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_localisation = get_package_share_directory("lunabot_localisation")

    rtabmap_yaml = os.path.join(pkg_localisation, "config", "rtabmap.yaml")
    ekf_yaml = os.path.join(pkg_localisation, "config", "ekf.yaml")
    apriltag_yaml = os.path.join(pkg_localisation, "config", "apriltag.yaml")

    stereo_remappings = [
        ("left/image_rect", "/camera_front_left"),
        ("right/image_rect", "/camera_front_right"),
        ("left/camera_info", "/camera_front_left/camera_info_synthetic"),
        ("right/camera_info", "/camera_front_right/camera_info_synthetic"),
    ]

    return LaunchDescription(
        [
            Node(
                package="lunabot_localisation",
                executable="stereo_camera_info_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "width": 640,
                        "height": 400,
                        "hfov": 1.396263402,
                        "baseline": 0.075,
                        "left_image_topic": "/camera_front_left",
                        "right_image_topic": "/camera_front_right",
                        "left_camera_info_topic": "/camera_front_left/camera_info_synthetic",
                        "right_camera_info_topic": "/camera_front_right/camera_info_synthetic",
                        "left_frame_id": "camera_front_left_optical_frame",
                        "right_frame_id": "camera_front_right_optical_frame",
                    }
                ],
            ),
            # Visual odometry
            Node(
                package="rtabmap_odom",
                executable="stereo_odometry",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[
                    *stereo_remappings,
                    ("odom", "/visual_odometry"),
                ],
            ),
            Node(
                package="lunabot_localisation",
                executable="visual_odometry_gate",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_topic": "/visual_odometry",
                        "odom_info_topic": "/odom_info",
                        "cmd_vel_topic": "/cmd_vel",
                        "gated_odom_topic": "/visual_odometry/gated",
                        "health_topic": "/visual_odometry/healthy",
                        "min_inliers": 20,
                        "min_matches": 40,
                        "max_position_variance": 0.25,
                        "max_yaw_variance": 0.25,
                        "odom_info_timeout_sec": 3.0,
                        "transition_log_interval_sec": 2.0,
                    }
                ],
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
                remappings=[
                    *stereo_remappings,
                    ("odom", "/odometry/filtered"),
                ],
                arguments=["-d"],
            ),
            # AprilTag detector
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                output="screen",
                parameters=[apriltag_yaml, {"use_sim_time": True}],
                remappings=[
                    ("image_rect", "/camera_front_left"),
                    ("camera_info", "/camera_front_left/camera_info_synthetic"),
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
                parameters=[
                    {
                        "use_sim_time": True,
                        "camera_frame": "camera_front_left_optical_frame",
                    }
                ],
            ),
        ]
    )
