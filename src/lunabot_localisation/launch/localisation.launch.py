import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate a launch description for the localisation stack.

    Start the local EKF in all modes, optionally publish the debug identity
    map->odom transform, and enable AprilTag/global EKF nodes by default.
    Visual SLAM remains available behind an explicit launch argument.
    """
    pkg_localisation = get_package_share_directory("lunabot_localisation")

    rtabmap_yaml = os.path.join(pkg_localisation, "config", "rtabmap.yaml")
    ekf_yaml = os.path.join(pkg_localisation, "config", "ekf.yaml")
    ekf_lidar_phase_yaml = os.path.join(
        pkg_localisation, "config", "ekf_lidar_phase.yaml"
    )
    apriltag_yaml = os.path.join(pkg_localisation, "config", "apriltag.yaml")
    lidar_costmap_phase = LaunchConfiguration("lidar_costmap_phase")
    enable_visual_slam = LaunchConfiguration("enable_visual_slam")

    visual_slam_condition = IfCondition(
        PythonExpression(
            [
                "'",
                lidar_costmap_phase,
                "' == 'false' and '",
                enable_visual_slam,
                "' == 'true'",
            ]
        )
    )

    camera_remappings = [
        ("rgb/image", "/camera_front/image"),
        ("depth/image", "/camera_front/depth_image"),
        ("rgb/camera_info", "/camera_front/camera_info"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_costmap_phase",
                default_value="false",
                description="Use odom-only debug localisation with an identity map->odom TF.",
            ),
            DeclareLaunchArgument(
                "enable_visual_slam",
                default_value="false",
                description=(
                    "Optionally enable RTAB-Map visual odometry "
                    "alongside AprilTag global localisation."
                ),
            ),
            # Visual odometry
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[*camera_remappings, ("odom", "/visual_odometry")],
                condition=visual_slam_condition,
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
            # During the lidar debug phase there is no global localisation source,
            # so publish an identity map->odom transform to keep Nav2 / RViz happy.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "odom",
                ],
                condition=IfCondition(lidar_costmap_phase),
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
            # RTAB-Map SLAM (optional, map building only, no TF publishing)
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": True}],
                remappings=[*camera_remappings, ("odom", "/odometry/local")],
                arguments=["-d"],
                condition=visual_slam_condition,
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
                name="tag_pose_publisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=UnlessCondition(lidar_costmap_phase),
            ),
        ]
    )
