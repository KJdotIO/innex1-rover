import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_localisation = get_package_share_directory("lunabot_localisation")

    rtabmap_yaml = os.path.join(pkg_localisation, "config", "rtabmap.yaml")
    ekf_yaml = os.path.join(pkg_localisation, "config", "ekf.yaml")
    ekf_lidar_phase_yaml = os.path.join(
        pkg_localisation, "config", "ekf_lidar_phase.yaml"
    )
    apriltag_yaml = os.path.join(pkg_localisation, "config", "apriltag.yaml")
    lidar_costmap_phase = LaunchConfiguration("lidar_costmap_phase")

    camera_remappings = [
        ("rgb/image", "/camera_front/image"),
        ("depth/image", "/camera_front/depth_image"),
        ("rgb/camera_info", "/camera_front/camera_info"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_costmap_phase",
                default_value="true",
                description="Launch only the local odom EKF for lidar/Nav2 bringup.",
            ),
            # Visual odometry
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[*camera_remappings, ("odom", "/visual_odometry")],
                condition=UnlessCondition(lidar_costmap_phase),
            ),
            # Local EKF: odom -> base_footprint (smooth, continuous)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[ekf_lidar_phase_yaml, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "/odometry/local")],
            ),
            # Global EKF: map -> odom (corrects drift when tag seen)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[ekf_yaml, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "/odometry/global")],
                condition=UnlessCondition(lidar_costmap_phase),
            ),
            # RTAB-Map SLAM (map building only, no TF publishing)
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[*camera_remappings, ("odom", "/odometry/local")],
                arguments=["-d"],
                condition=UnlessCondition(lidar_costmap_phase),
            ),
            # AprilTag detector
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                parameters=[apriltag_yaml, {"use_sim_time": True}],
                remappings=[
                    ("image_rect", "/camera_front/image"),
                    ("camera_info", "/camera_front/camera_info"),
                ],
                condition=UnlessCondition(lidar_costmap_phase),
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
                condition=UnlessCondition(lidar_costmap_phase),
            ),
            # Converts apriltag TF into PoseWithCovarianceStamped for global EKF
            Node(
                package="lunabot_localisation",
                executable="tag_pose_publisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=UnlessCondition(lidar_costmap_phase),
            ),
        ]
    )
