import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch localisation: wheel odom + IMU EKF backbone + AprilTag correction.

    Visual odometry is intentionally disabled. Wheel odom + IMU provide
    smooth continuous pose; AprilTag gives absolute map-frame corrections
    to kill accumulated drift. RTAB-Map builds the occupancy map for Nav2
    using the EKF-fused odom (no VO node needed).
    """
    pkg_localisation = get_package_share_directory("lunabot_localisation")

    rtabmap_yaml = os.path.join(pkg_localisation, "config", "rtabmap.yaml")
    ekf_yaml = os.path.join(pkg_localisation, "config", "ekf.yaml")
    apriltag_yaml = os.path.join(pkg_localisation, "config", "apriltag.yaml")

    return LaunchDescription(
        [
            # ──────────────────────────────────────────────────────────
            # RGB-D stream republisher
            # Gazebo bridge can deliver delayed / out-of-order timestamps.
            # Re-stamp streams with monotonic "now" and drop stale frames.
            # ──────────────────────────────────────────────────────────
            Node(
                package="lunabot_localisation",
                executable="rgbd_stream_republisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_image_topic": "/camera_front/image",
                        "input_depth_topic": "/camera_front/depth_image",
                        "input_camera_info_topic": "/camera_front/camera_info",
                        "output_image_topic": "/camera_front/image_sync",
                        "output_depth_topic": "/camera_front/depth_image_sync",
                        "output_camera_info_topic": "/camera_front/camera_info_sync",
                        "max_input_age_sec": 0.0,
                        "publish_info_on_depth": True,
                    }
                ],
            ),
            # ──────────────────────────────────────────────────────────
            # Local EKF: odom → base_footprint (smooth, continuous)
            # Wheel odom + IMU only. No visual odometry.
            # ──────────────────────────────────────────────────────────
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True}],
            ),
            # ──────────────────────────────────────────────────────────
            # Global EKF: map → odom (jumps when AprilTag seen)
            # ──────────────────────────────────────────────────────────
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "/odometry/global")],
            ),
            # ──────────────────────────────────────────────────────────
            # RTAB-Map SLAM (map building only, no VO, no TF publishing)
            # Fed the EKF-fused odometry for pose graph backbone.
            # Images are used for loop closure + occupancy grid only.
            # ──────────────────────────────────────────────────────────
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[
                    ("rgb/image", "/camera_front/image_sync"),
                    ("depth/image", "/camera_front/depth_image_sync"),
                    ("rgb/camera_info", "/camera_front/camera_info_sync"),
                    ("odom", "/odometry/filtered"),
                ],
                arguments=["-d"],
            ),
            # ──────────────────────────────────────────────────────────
            # AprilTag detector
            # ──────────────────────────────────────────────────────────
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                output="screen",
                parameters=[apriltag_yaml, {"use_sim_time": True}],
                remappings=[
                    ("image_rect", "/camera_front/image_sync"),
                    ("camera_info", "/camera_front/camera_info_sync"),
                    ("/camera_front/camera_info", "/camera_front/camera_info_sync"),
                    ("image_rect/camera_info", "/camera_front/camera_info_sync"),
                    ("/camera_front/image_sync/camera_info", "/camera_front/camera_info_sync"),
                ],
            ),
            # ──────────────────────────────────────────────────────────
            # Static TF: known position of tag 0 in the map frame
            # ──────────────────────────────────────────────────────────
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0",
                    "--y", "1.08",
                    "--z", "0.12",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "tag36h11:0",
                ],
            ),
            # ──────────────────────────────────────────────────────────
            # AprilTag → PoseWithCovarianceStamped for global EKF
            # ──────────────────────────────────────────────────────────
            Node(
                package="lunabot_localisation",
                executable="tag_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "camera_frame": "camera_front_link",
                    }
                ],
            ),
            # ──────────────────────────────────────────────────────────
            # Topic Health Watchdog
            # Monitors key topic rates and logs warnings when any
            # critical source drops below threshold.
            # ──────────────────────────────────────────────────────────
            Node(
                package="lunabot_localisation",
                executable="topic_health_watchdog",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
