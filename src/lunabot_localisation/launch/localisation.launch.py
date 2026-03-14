import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch localisation: EKF backbone + supplementary RGB-D VO + AprilTag."""
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
            # Depth -> PointCloud for Nav2 obstacle layers
            # The native Gazebo /camera_front/points bridge is bursty.
            # Build a dense, stable cloud directly from depth + intrinsics.
            # ──────────────────────────────────────────────────────────
            Node(
                package="rtabmap_util",
                executable="point_cloud_xyz",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.25,
                        "topic_queue_size": 20,
                        "sync_queue_size": 20,
                        "decimation": 2,
                        "voxel_size": 0.03,
                        "min_depth": 0.15,
                        "max_depth": 4.0,
                        "filter_nans": True,
                    }
                ],
                remappings=[
                    ("depth/image", "/camera_front/depth_image_sync"),
                    ("depth/camera_info", "/camera_front/camera_info_sync"),
                    ("cloud", "/camera_front/points_nav"),
                ],
            ),
            # ──────────────────────────────────────────────────────────
            # RGB-D Visual Odometry (supplementary)
            # ──────────────────────────────────────────────────────────
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[
                    ("rgb/image", "/camera_front/image_sync"),
                    ("depth/image", "/camera_front/depth_image_sync"),
                    ("rgb/camera_info", "/camera_front/camera_info_sync"),
                    ("odom", "/visual_odometry"),
                ],
            ),
            # ──────────────────────────────────────────────────────────
            # Visual Odometry Gate
            # Only passes VO to EKF when tracking is healthy.
            # Sensor contract health is also required (published by watchdog).
            # ──────────────────────────────────────────────────────────────────
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
                        "sensor_health_topic": "/diagnostics/topics_healthy",
                        "require_sensor_health": False,
                        "min_inliers": 10,
                        "min_matches": 20,
                        "max_position_variance": 1.0,
                        "max_yaw_variance": 1.0,
                        "odom_info_timeout_sec": 6.0,
                        "sensor_health_timeout_sec": 12.0,
                        "transition_log_interval_sec": 5.0,
                    }
                ],
            ),
            # ──────────────────────────────────────────────────────────
            # Local EKF: odom → base_footprint (smooth, continuous)
            # BACKBONE: wheel odom + IMU. VO is supplementary.
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
            # RTAB-Map SLAM (map building only, no TF publishing)
            # Fed the EKF-fused odometry so it gets a stable odom source
            # even when VO is lost.
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
                    "--z", "0.25",
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
