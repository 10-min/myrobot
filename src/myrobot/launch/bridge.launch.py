from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', default_value='ros_gz_bridge', description='Name of ros_gz_bridge node'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', 
        default_value=PathJoinSubstitution([
            FindPackageShare('myrobot'),
            'config',
            'bridge.yaml'
        ]),
        description='YAML config file'
    )
    
    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')
    bridge_node = RosGzBridge(
        bridge_name=bridge_name,
        config_file=config_file
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(bridge_node)

    return ld