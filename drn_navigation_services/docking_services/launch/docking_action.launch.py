from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='docking_services',
            executable='docking_services_node',
            output='screen'),
    ])