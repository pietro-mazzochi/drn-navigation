from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_name = LaunchConfiguration('map_name')

    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='map',
        description='Name of the map, speeds and keepout zones files.'
    )

    pkg_dir = get_package_share_directory('drn_navigation_mapping')

    controller_yaml = os.path.join(pkg_dir, 'config', 'controller.yaml')
    planner_yaml = os.path.join(pkg_dir, 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(pkg_dir, 'config', 'recovery.yaml')
    bt_navigator_yaml = os.path.join(pkg_dir, 'config', 'bt_navigator.yaml')
    config_keepout_yaml = os.path.join(pkg_dir, 'config', 'filter_keepout.yaml')
    config_speeds_yaml = os.path.join(pkg_dir, 'config', 'filter_speeds.yaml')

    keepout_file = PathJoinSubstitution([FindPackageShare('drn_navigation_mapping'), 'maps', 'map_keepout', PythonExpression(["'", map_name, "' + '.yaml'"])])
    speeds_file = PathJoinSubstitution([FindPackageShare('drn_navigation_mapping'), 'maps', 'map_speeds', PythonExpression(["'", map_name, "' + '.yaml'"])])

    pose_bt_xml_path = os.path.join(pkg_dir, 'config', 'dalca_behavior_pose.xml')
    poses_bt_xml_path = os.path.join(pkg_dir, 'config', 'dalca_behavior_poses.xml')

    bt_navigator_updated_config = RewrittenYaml(
        source_file=bt_navigator_yaml,
        root_key='',
        param_rewrites={
            "default_nav_to_pose_bt_xml": pose_bt_xml_path,
            "default_nav_through_poses_bt_xml": poses_bt_xml_path,
        },
        convert_types=True
    )

    global_costmap_serv_launch_path = os.path.join(
        get_package_share_directory('global_costmap_service'),
        'launch',
        'get_cost_service.launch.py'
    )

    return LaunchDescription([
        map_name_arg,

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='log', #screen',
            remappings=[('/cmd_vel', '/mrac_controller/command')],
            parameters=[{'use_sim_time': use_sim_time}, controller_yaml]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='log', #screen',
            parameters=[{'use_sim_time': use_sim_time}, planner_yaml]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='log', #screen',
            remappings=[('/cmd_vel', '/mrac_controller/command')],
            parameters=[{'use_sim_time': use_sim_time}, recovery_yaml]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='log', #screen',
            parameters=[{'use_sim_time': use_sim_time}, bt_navigator_updated_config]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='log', #screen',
            emulate_tty=True,
            parameters=[config_keepout_yaml, {'use_sim_time': use_sim_time}, {'yaml_filename': keepout_file}]
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='log', #screen',
            emulate_tty=True,
            parameters=[config_keepout_yaml, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='speeds_filter_mask_server',
            output='log', #screen',
            emulate_tty=True,
            parameters=[config_speeds_yaml, {'use_sim_time': use_sim_time}, {'yaml_filename': speeds_file}]
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speeds_costmap_filter_info',
            output='log', #screen',
            emulate_tty=True,
            parameters=[config_speeds_yaml, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='log', #screen',
            parameters=[
                {'autostart': True},               
                #{'use_lifecycle_mgr': False },
                #{'bond_timeout': 5.0 },
                {'attempt_recovery': False },
                {'node_names': [
                    "filter_mask_server",
                    "costmap_filter_info_server",
                    "speeds_filter_mask_server",
                    "speeds_costmap_filter_info",
                    "planner_server",
                    "controller_server",
                    "recoveries_server",
                    "bt_navigator"
                ]}
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(global_costmap_serv_launch_path)
        )
    ])