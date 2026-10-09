import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():  
    return LaunchDescription([

        DeclareLaunchArgument(
            'slam_node_name',
            default_value='drn_navigation_mapping_node',
            description='Name of the SLAM node.'
        ),

        Node(
        package='drn_navigation_mapping',
        executable='drn_navigation_mapping_node',
        name=LaunchConfiguration('slam_node_name'),
        namespace = "drn_navigation/mapping",
        output='log', #screen',
        emulate_tty=True,
        parameters=[{"param1": "Test"},{"param2": 1000}],
        remappings=[('/input/slam_toggle', 
                     'slam_toggle'),
            ]
        )
    ])