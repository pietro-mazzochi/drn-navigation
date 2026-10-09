from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_name = LaunchConfiguration('map_name')

    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='map',
        description='Name of the map, speeds and keepout zones files.'
    )

    map_file = PathJoinSubstitution([
        FindPackageShare('drn_navigation_mapping'), 'maps', 'map', PythonExpression(["'", map_name, "' + '.yaml'"])
    ])

    amcl_yaml = PathJoinSubstitution([
        FindPackageShare('drn_navigation_mapping'), 'config', 'amcl.yaml'
    ])

    return LaunchDescription([
        map_name_arg,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file}
            ]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='log', #screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                amcl_yaml
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='log', #screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                #{'bond_timeout': 5.0 }, 
                #{'use_lifecycle_mgr': False },
                {'attempt_recovery': False },
                {'node_names': [
                    'map_server', 
                    'amcl'
                    ]}
            ]
        ),
    ])