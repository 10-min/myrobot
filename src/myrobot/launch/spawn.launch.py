from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_my_robot = get_package_share_directory('myrobot')

    urdf_file = os.path.join(pkg_my_robot, 'urdf', 'myrobot.urdf.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    def robot_state_publisher(context):
        performed_description_format = LaunchConfiguration('description_format').perform(context)
        # Get URDF or SDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('myrobot'),
                    performed_description_format,
                    f'myrobot.xacro.{performed_description_format}'
                ]),
            ]
        )
        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('gz_ros2_control_demos'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description', '-name',
                    'diff_drive', '-allow_renaming', 'true'],
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
        