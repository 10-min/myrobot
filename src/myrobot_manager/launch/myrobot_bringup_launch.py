import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.substitutions
from launch_ros.actions import Node
import launch_ros
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    pkg_name = 'myrobot_manager'

    pkg_dir = get_package_share_directory(pkg_name)
    config_dir = os.path.join(pkg_dir, 'config')
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    
    myrobot_description_launch_path = os.path.join(get_package_share_directory('myrobot_description'), 'launch', 'display_launch.py')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Flag to enable use_sim_time'
    )
    
    myrobot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(myrobot_description_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    myrobot_manager_node = Node(
        package=pkg_name,
        executable='myrobot_manager',
        name='myrobot_manager',
        output='screen',
    )
    
    myrobot_imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'imu_filter.yaml')],
        remappings=[
            ('/imu/data', '/imu')
        ]
    )
    
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch','sllidar_c1_launch.py')
        ),
        launch_arguments={
            'frame_id': "lidar_link",
        }.items(),
    )
    
    myrobot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    return launch.LaunchDescription(
        [
            myrobot_description_launch,
            myrobot_manager_node,
            myrobot_imu_filter_node,
            
            sllidar_launch,
            myrobot_localization_node,
        ]
    )