from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='global_costmap_service',
            executable='get_cost_service_node',
            output='screen'),
    ])