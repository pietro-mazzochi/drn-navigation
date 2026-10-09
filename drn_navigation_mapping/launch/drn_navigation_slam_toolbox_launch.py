from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch parameters
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Arguments
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('drn_navigation_mapping'),
            'config',
            'slam.yaml'
        ]),
        description='Path to the SLAM Toolbox parameters YAML file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto-start lifecycle nodes'
    )

    # SLAM Toolbox node with remappings for only published topics
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='log', #screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            {'autostart': autostart}
        ],
        #remappings=[
        #    ('/map', '/meu_mapa'),
        #    ('/map_metadata', '/meu_mapa_metadata'),
        #    ('/tf', '/meu_tf')
        #]
    )

    return LaunchDescription([
        declare_slam_params_file,
        declare_use_sim_time,
        declare_autostart,
        slam_toolbox_node,
    ])
