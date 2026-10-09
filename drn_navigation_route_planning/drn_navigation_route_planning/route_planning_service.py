import networkx as nx
import json
from copy import deepcopy
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped

from drn_navigation_msgs.msg import Edge
from drn_navigation_msgs.srv import GraphPath
from drn_navigation_route_planning.route_planning import Graph

class RoutePlanningService(Node):

    def __init__(self):
        super().__init__("route_planning_service")
        self.graph = Graph()

        self.declare_parameter("robot_step_size", 1.0)
        self.robot_step_size = self.get_parameter("robot_step_size").value

        self.declare_parameter("node", "[]")
        self.declare_parameter("node_type", "[]")
        self.declare_parameter("node_connection", "[]")

        self.node_data = json.loads(self.get_parameter("node").value)
        self.node_type_data = json.loads(self.get_parameter("node_type").value)
        self.node_connection_data = json.loads(self.get_parameter("node_connection").value)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.srv = self.create_service(
            GraphPath,
            "calculate_global_steps_service",
            self.plan_path,
        )

        self.update_graph()

    def parameter_callback(self, params):
        update_graph = False
        try:
            for param in params:
                self.get_logger().info(f"Parameter '{param.name}' has been updated.")
                match param.name:
                    case "node":
                        self.node_data = json.loads(param.value)
                        update_graph = True
                    case "node_connection":
                        self.node_connection_data = json.loads(param.value)
                        update_graph = True
                    case "node_type":
                        self.node_type_data = json.loads(param.value)
                        update_graph = True
                    case "robot_step_size":
                        self.robot_step_size = param.value
                    case _:
                        self.get_logger().info(f"Parameter '{param.name}' changed, but no specific action is defined.")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON parameter: {e}")
            return SetParametersResult(successful=False)

        if update_graph:
            self.update_graph()

        return SetParametersResult(successful=True)
    
    def update_graph(self):
        self.get_logger().info("Route Planning updating graph...")

        if not self.node_data or not self.node_connection_data or not self.node_type_data:
            self.get_logger().warn("Graph data is incomplete. Skipping graph update.")
            return
        try:
            self.graph = Graph()
            self.graph.add_nodes_from_json(json.dumps(self.node_data))
            self.graph.add_edges_from_json(json.dumps(self.node_connection_data))

            node_type_map = {data["id"]: data["name"] for data in self.node_type_data}

            empty_nodes = [
                id for id, attrs in self.graph.nodes(data=True) if "pos_x" not in attrs
            ]
            self.graph.remove_nodes_from(empty_nodes)

            for _, node in self.graph.nodes(data=True):
                node["node_type"] = node_type_map.get(node["node_type_id"], "Desconhecido")

            self.get_logger().info(f"Graph: {self.graph.nodes(data=True)}")
        except:
            self.get_logger().warn("Graph data is incorrect. Skipping graph update.")
            return


    def plan_path(self, request, response):
        self.get_logger().info("Route Planning Service Running")

        if len(self.graph.nodes()) == 0 or len(self.graph.edges()) == 0:
            self.get_logger().error("Graph is empty. Check 'node', 'node_type' and 'node_connection' parameters.")
            response.path = []
            response.relative_edges = []
            response.dock_robot = False
            return response
    
        self.get_logger().info(
            f"Nodes: {len(list(self.graph.nodes()))} Edges: {len(list(self.graph.edges()))}"
        )
        response.dock_robot = False
        robot_position = request.robot_pose.pose.position
        robot_orientation = request.robot_pose.pose.orientation
        new_graph, robot_position_id = self.graph.generate_route_to_robot(
            (robot_position.x, robot_position.y), self.robot_step_size
        )
        new_graph.set_distances()
        quat = (
            robot_orientation.x,
            robot_orientation.y,
            robot_orientation.z,
            robot_orientation.w,
        )
        try:
            path = new_graph.find_path(
                robot_position_id,
                request.id_target,
                blocked_paths=[
                    (edge.id_source, edge.id_target) for edge in request.blocked_paths
                ],
            )
        except nx.NodeNotFound:
            self.get_logger().info("NodeNotFound")
            response.path = []
        except nx.NetworkXNoPath:
            self.get_logger().info("NetworkXNoPath")
            response.path = []
        else:
            if (new_graph.nodes[path[-1]]["node_type"] == "Doca") and (
                new_graph.nodes[path[-2]]["node_type"] == "Detecção"
            ):
                path = deepcopy(path[:-1])
                response.dock_robot = True
            path = new_graph.generate_intermediate_steps(
                path, alpha=self.robot_step_size
            )
            if request.direction == 'front':
                pass
            elif request.direction == 'rear':
                self.get_logger().warn(f"inverted path")
                path = [(step, edge, yaw + np.pi) for (step, edge, yaw) in path]
            yaw_diff = quaternion_to_yaw(quat) - path[0][2]
            norm_yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
            if abs(norm_yaw_diff) > np.pi / 2 and len(path) >= 3:
                mid = len(path) // 2
                initial_half_path = [(step, edge, yaw + np.pi) for (step, edge, yaw) in path[:mid]]
                final_half_path = path[mid:]
                path = initial_half_path + final_half_path
                self.get_logger().warn(f"inverted half path")
            response.path, response.relative_edges = self.generate_stamped_poses(path)

        return response

    def generate_stamped_poses(self, path):
        route_poses = []
        route_poses_edge = []
        pose = PoseStamped()
        pose_edge = Edge()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        for step, edge, orientation in path:
            pose.pose.position.x = step[0]
            pose.pose.position.y = step[1]
            quaternion_orientation = yaw_to_quaternion(orientation)
            pose.pose.orientation.x = quaternion_orientation[0]
            pose.pose.orientation.y = quaternion_orientation[1]
            pose.pose.orientation.z = quaternion_orientation[2]
            pose.pose.orientation.w = quaternion_orientation[3]
            route_poses.append(deepcopy(pose))

            pose_edge.id_source = edge[0]
            pose_edge.id_target = edge[1]
            route_poses_edge.append(deepcopy(pose_edge))
        return route_poses, route_poses_edge


def yaw_to_quaternion(yaw):
    """Converte um ângulo yaw em quaternion."""
    x = 0.0
    y = 0.0
    z = np.sin(yaw / 2)
    w = np.cos(yaw / 2)
    return (x, y, z, w)


def quaternion_to_yaw(quaternion):
    x, y, z, w = quaternion
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return yaw


def main():
    rclpy.init()

    rps = RoutePlanningService()
    rclpy.spin(rps)

    rps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
