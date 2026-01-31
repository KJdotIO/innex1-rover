# // MODULES // 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# // LAUNCH FUNCTION // 
def generate_launch_description():
    # 1. find YAML file path and fetch nav2 params from it 
    nav2_params = PathJoinSubstitution([
    FindPackageShare('lunabot_navigation'),
    'config',
    'nav2_params.yaml'
    ])
    
    # 2. startup nodes using the official nav2 bringup launch file 
    # (bypasses the need to explicitly load each Nav2 Node)
    # https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/launch/navigation_launch.py 

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        
        # 3. load YAML parameters + force use_sim_time = true
        launch_arguments={
        'params_file': nav2_params,
        'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
