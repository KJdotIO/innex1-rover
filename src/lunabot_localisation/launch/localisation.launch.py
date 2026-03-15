import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate a launch description for the localisation stack.

    Launches the RGB-D republisher, dual EKFs, RTAB-Map, AprilTag detection,
    tag pose bridge, topic health watchdog, and status dashboard.
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
                arguments=["--ros-args", "--log-level", "rgbd_stream_republisher:=warn"],
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_image_topic": "/camera_front/image",
                        "input_depth_topic": "/camera_front/depth_image",
                        "input_camera_info_topic": "/camera_front/camera_info",
                        "output_image_topic": "/camera_front/image_sync",
                        "output_depth_topic": "/camera_front/depth_image_sync",
                        "output_camera_info_topic": "/camera_front/camera_info_sync",
                        "max_input_age_sec": 0.5,
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
                arguments=["-d", "--ros-args", "--log-level", "rtabmap:=warn"],
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
                    "--x", "-2.95",
                    "--y", "2.15",
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
            # Topic Health Watchdog (WARN-only: silent unless degraded)
            # ──────────────────────────────────────────────────────────
            Node(
                package="lunabot_localisation",
                executable="topic_health_watchdog",
                output="screen",
                arguments=["--ros-args", "--log-level", "topic_health_watchdog:=warn"],
                parameters=[
                    {
                        "use_sim_time": True,
                    }
                ],
            ),
            # ──────────────────────────────────────────────────────────
            # Status Dashboard: compact one-line periodic summary
            # ──────────────────────────────────────────────────────────
            Node(
                package="lunabot_localisation",
                executable="status_dashboard",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "period_sec": 5.0,
                    }
                ],
            ),
        ]
    )
