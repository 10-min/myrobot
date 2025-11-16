from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',  # 패키지 이름 변경
            executable='odom_echo',       # 위 노드 이름
            name='odom_echo_node',
            output='screen'
        )
    ])