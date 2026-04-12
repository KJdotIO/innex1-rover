from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch the Point-LIO RViz profile.",
    )

    laser_mapping_params = [
        PathJoinSubstitution(
            [FindPackageShare("point_lio"), "config", "marsim.yaml"]
        ),
        {
            "use_imu_as_input": False,
            "prop_at_freq_of_imu": True,
            "check_satu": True,
            "init_map_size": 10,
            "point_filter_num": 1,
            "space_down_sample": True,
            "filter_size_surf": 0.2,
            "filter_size_map": 0.2,
            "cube_side_length": 50.0,
            "runtime_pos_log_enable": False,
            "odom_only": True,
            "odom_header_frame_id": "odom",
            "odom_child_frame_id": "base_footprint",
        },
    ]

    laser_mapping_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=laser_mapping_params,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("point_lio"), "rviz_cfg", "loam_livox.rviz"]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        prefix="nice",
    )

    return LaunchDescription(
        [
            rviz_arg,
            laser_mapping_node,
            GroupAction(
                actions=[rviz_node],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
