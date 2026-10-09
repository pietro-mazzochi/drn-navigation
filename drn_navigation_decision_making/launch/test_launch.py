from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Launch the decision_maker_node
    decision_maker_node = Node(
        package='drn_navigation_decision_maker',
        executable='drn_navigation_decision_maker_node',
        name='drn_navigation_decision_maker_node',
        namespace="drn_navigation/decision_making",
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
        ],
    )

    # Launch the mock test script
    test_node = Node(
        package='drn_navigation_decision_maker',
        executable='test_decision_maker_node.py',
        name='test_decision_maker',
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        decision_maker_node,
        test_node
    ])
