import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
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
    tag_pose_bridge_yaml = os.path.join(
        pkg_localisation, "config", "tag_pose_bridge.yaml"
    )
    tag_pose_bridge_sim_yaml = os.path.join(
        pkg_localisation, "config", "tag_pose_bridge_sim.yaml"
    )
    start_zone_localisation_yaml = os.path.join(
        pkg_localisation, "config", "start_zone_localisation.yaml"
    )
    lidar_costmap_phase = LaunchConfiguration("lidar_costmap_phase")
    enable_visual_slam = LaunchConfiguration("enable_visual_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_apriltag_debug = LaunchConfiguration("enable_apriltag_debug")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    tag_map_x = LaunchConfiguration("tag_map_x")
    tag_map_y = LaunchConfiguration("tag_map_y")
    tag_map_z = LaunchConfiguration("tag_map_z")
    tag_map_yaw = LaunchConfiguration("tag_map_yaw")

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
    apriltag_debug_condition = IfCondition(
        PythonExpression(
            [
                "'",
                lidar_costmap_phase,
                "' == 'false' and '",
                enable_apriltag_debug,
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
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock instead of wall time for all launched nodes.",
            ),
            DeclareLaunchArgument(
                "enable_apriltag_debug",
                default_value="false",
                description=(
                    "Launch the apriltag_draw overlay for annotated front camera "
                    "debugging."
                ),
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="cmd_vel",
                description="Velocity command topic used by the start-zone localiser.",
            ),
            DeclareLaunchArgument(
                "tag_map_x",
                default_value="0.0",
                description="Configured map-frame x position of the start-zone tag.",
            ),
            DeclareLaunchArgument(
                "tag_map_y",
                default_value="1.08",
                description="Configured map-frame y position of the start-zone tag.",
            ),
            DeclareLaunchArgument(
                "tag_map_z",
                default_value="0.25",
                description="Configured map-frame z position of the start-zone tag.",
            ),
            DeclareLaunchArgument(
                "tag_map_yaw",
                default_value="0.0",
                description="Configured map-frame yaw of the start-zone tag.",
            ),
            # Visual odometry
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": use_sim_time}],
                remappings=[*camera_remappings, ("odom", "/visual_odometry")],
                condition=visual_slam_condition,
            ),
            # Local EKF: odom -> base_footprint (smooth, continuous)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[ekf_lidar_phase_yaml, {"use_sim_time": use_sim_time}],
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
                parameters=[ekf_yaml, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odometry/global")],
                condition=UnlessCondition(lidar_costmap_phase),
            ),
            # RTAB-Map SLAM (optional, map building only, no TF publishing)
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[rtabmap_yaml, {"use_sim_time": use_sim_time}],
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
                parameters=[apriltag_yaml, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("image_rect", "/camera_front/image"),
                    ("camera_info", "/camera_front/camera_info"),
                    ("detections", "/camera_front/tags"),
                ],
                condition=UnlessCondition(lidar_costmap_phase),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("apriltag_draw"), "launch", "draw.launch.py"]
                    )
                ),
                launch_arguments={
                    "camera": "/camera_front",
                    "image": "image",
                    "tags": "tags",
                    "image_transport": "raw",
                }.items(),
                condition=apriltag_debug_condition,
            ),
            # Known position of tag 0 in the map frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", tag_map_x,
                    "--y", tag_map_y,
                    "--z", tag_map_z,
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", tag_map_yaw,
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
                parameters=[
                    tag_pose_bridge_sim_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "detections_topic": "/camera_front/tags",
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            lidar_costmap_phase,
                            "' == 'false' and '",
                            use_sim_time,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="lunabot_localisation",
                executable="tag_pose_publisher",
                name="tag_pose_publisher",
                output="screen",
                parameters=[
                    tag_pose_bridge_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "detections_topic": "/camera_front/tags",
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            lidar_costmap_phase,
                            "' == 'false' and '",
                            use_sim_time,
                            "' != 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="lunabot_localisation",
                executable="start_zone_localiser",
                name="start_zone_localiser",
                output="screen",
                parameters=[
                    start_zone_localisation_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "cmd_vel_topic": cmd_vel_topic,
                    },
                ],
                condition=UnlessCondition(lidar_costmap_phase),
            ),
        ]
    )
