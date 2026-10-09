#from launch import LaunchContext, parse_launch_arguments
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drn_navigation_decision_maker',
            executable='drn_navigation_decision_maker_node',
            name='drn_navigation_decision_maker_node',
            namespace = "drn_navigation/decision_making",
            output="screen",
            emulate_tty=True,
            parameters=[
            {"param1": "Test"},
            {"param2": 1000},
            ],
            remappings=[
                # topics
                ('/input/rgb', 'rgb'),
                ('/input/dock_detector', '/drn_perception/dock_detector/dock_detection'),
                ('/output/alert_service', '/drn_integration/communication/alert_service'),
                ('/output/log_service', '/drn_integration/communication/log_service'),
                ('/output/mission_log_service', '/drn_integration/communication/mission_log_service'),
            ]
        )
    ])