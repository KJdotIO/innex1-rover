from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory





def generate_launch_description():
    ekf_config_path = PathJoinSubstitution([
        get_package_share_directory('lunabot_localisation'),
        'config',
        'ekf.yaml'
    ])
    return LaunchDescription([
        # 1. Odometry Node
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{'publish_tf': False}],
            remappings=[('/odom', '/visual_odometry')]
        ),
        Node(
            package='robot_localization', executable='ekf_node', output='screen',
            parameters=[ekf_config_path]
            
        ),
        # 2. SLAM Node
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{'Grid/CellSize': '0.05'}],
            arguments=['-d'],
            remappings=[('/odom', '/odometry/filtered')]
        )
    ])
