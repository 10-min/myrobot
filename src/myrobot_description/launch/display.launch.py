from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_sim.actions import GzServer
from ros_gz_bridge.actions import RosGzBridge
import launch
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='myrobot_description').find('myrobot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'myrobot.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    world_path = os.path.join(pkg_share, 'worlds', 'test.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge.yaml')
    
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_model_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot model file'
    )
    declare_rvizconfig_cmd = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Flag to enable use_sim_time'
    )
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(use_sim_time),
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model])}, {'use_sim_time': use_sim_time}]
    )
    
    
    rviz_node = Node(
        condition=IfCondition(use_sim_time),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
    )
    
    gz_server = GzServer(
        condition=IfCondition(use_sim_time),
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='true',
        use_composition='true',
    )
    
    ros_gz_bridge = RosGzBridge(
        condition=IfCondition(use_sim_time),
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='false',
        use_composition='true',
    )
    
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        condition=IfCondition(use_sim_time),
        launch_arguments={
            'world': 'hospital_world',
            'topic': '/robot_description',
            'entity_name': 'myrobot',
            'y': '-0.5',
            'z': '0.65',
        }.items(),
    )
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        declare_model_cmd,
        declare_rvizconfig_cmd,
        declare_use_sim_time_cmd,
        gz_sim,
        robot_state_publisher_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        robot_localization_node
    ])