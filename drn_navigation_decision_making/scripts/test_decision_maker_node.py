#!/usr/bin/env python3

# Mock test for the decision_maker_node package
# mocks the integration_communication node

import rclpy
from rclpy.node import Node
from drn_integration_msgs.msg import Mission
import time

class MockMissionPublisher(Node):
    def __init__(self):
        super().__init__('mock_mission_publisher')

        # Create a publisher for the /mission topic
        self.publisher_ = self.create_publisher(Mission, '/mission', 10)

        # Timer for periodic testing
        self.timer_ = self.create_timer(1.0, self.timer_callback)

        # Test scenarios
        self.test_scenarios = [
            {"command": "start", "id": 1, "end_id": 200},
            {"command": "pause", "id": 1, "end_id": 200},
            {"command": "continue", "id": 1, "end_id": 200},
            {"command": "cancel", "id": 1, "end_id": 200},
        ]
        self.current_scenario_index = 0

    def timer_callback(self):
        # Send the next test command
        if self.current_scenario_index < len(self.test_scenarios):
            scenario = self.test_scenarios[self.current_scenario_index]
            msg = Mission()
            msg.command = scenario["command"]
            msg.id = scenario["id"]
            msg.end_id = scenario["end_id"]

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published Mission: {msg}")

            self.current_scenario_index += 1
        else:
            self.get_logger().info("All test scenarios completed. Shutting down...")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    mock_node = MockMissionPublisher()

    try:
        rclpy.spin(mock_node)
    except KeyboardInterrupt:
        pass
    finally:
        mock_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
