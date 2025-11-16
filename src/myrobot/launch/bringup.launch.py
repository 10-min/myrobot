from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('myrobot')

    urdf_file = os.path.join(pkg_share, 'urdf', 'myrobot.urdf.xacro')

    robot_description_content = xacro.process_file(urdf_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = os.path.join(pkg_share, 'config', 'myrobot_ros2_control.yaml')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

    ])
