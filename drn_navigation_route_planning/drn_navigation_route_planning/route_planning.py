import numpy as np
import networkx as nx
import json
from copy import deepcopy


class Graph(nx.Graph):
    def __init__(self, incoming_graph_data=None, **kwargs):
        super().__init__(incoming_graph_data, **kwargs)

        if not self.graph:
            self.graph = {}

        for key, value in kwargs.items():
            self.graph[key] = value

    def find_path(
        self,
        source: str,
        target: str,
        weight="distance",
        blocked_paths: list[tuple[int, int]] = [],
    ) -> list[str]:
        """Find smallest path from source to target using dijkstra algorithm."""
        g = self.copy()
        for edge in blocked_paths:
            g.remove_edge(edge[0], edge[1])

        return nx.dijkstra_path(g, source=source, target=target, weight=weight)

    def set_distances(self, weight="distance", force=False):
        """Set the weight of the edges based on the distances between node position."""
        for source, target, attr in self.edges(data=True):
            if (force is not True) and (attr.get(weight, None) is not None):
                continue
            dx = float(self.nodes[source]["pos_x"]) - float(self.nodes[target]["pos_x"])
            dy = float(self.nodes[source]["pos_y"]) - float(self.nodes[target]["pos_y"])
            attr["distance"] = np.sqrt(dx**2 + dy**2)

    def generate_intermediate_steps(self, path, alpha=0.2) -> list:
        source = path.pop(0)
        path_steps = []
        for target in path:
            start = np.array((self.nodes[source]["pos_x"], self.nodes[source]["pos_y"]))
            end = np.array((self.nodes[target]["pos_x"], self.nodes[target]["pos_y"]))
            V = end - start
            V_norm = np.linalg.norm(V)

            if V_norm == 0:
                source = target
                continue

            V_dir = V / V_norm
            orientation = np.arctan2(V[1], V[0])
            for i in range(int(V_norm / alpha) + 1):
                next_step = np.around(
                    start + i * alpha * V_dir,
                    3,
                )
                path_steps.append(
                    (
                        tuple(next_step.tolist()),
                        (source, target),
                        orientation,
                    )
                )
            source = target
        path_steps.append(
            (
                tuple(end.tolist()),
                (source, target),
                orientation,
            )
        )
        return path_steps

    def add_nodes_from_json(self, json_data, index: str = "id"):
        data_list = json.loads(json_data)
        # Fixing position format to float
        for data in data_list:
            data["pos_x"] = float(data["pos_x"])
            data["pos_y"] = float(data["pos_y"])
        self.add_nodes_from((data[index], data) for data in data_list)

    def add_edges_from_json(
        self,
        json_data: str,
        source_key: str = "source_node_id",
        target_key: str = "target_node_id",
    ):
        edge_list = [
            (edge[source_key], edge[target_key]) for edge in json.loads(json_data)
        ]
        self.add_edges_from(edge_list)
        self.set_distances(force=True)

    def generate_route_to_robot(
        self, robot_position: list | tuple, robot_step_size: float
    ) -> tuple[nx.Graph, int]:
        graph_copy = deepcopy(self)
        closest_distance = float("inf")
        closest_distance_edge = (None, None)
        closest_distance_point = (None, None)
        robot_node_id = -1
        intermediary_node_id = -2

        node_pos = lambda g, n: np.array((g.nodes[n]["pos_x"], g.nodes[n]["pos_y"]))

        for edge in graph_copy.edges():
            A = node_pos(graph_copy, edge[0])
            B = node_pos(graph_copy, edge[1])

            P, dist = point_to_segment(A, B, robot_position)
            if dist < closest_distance:
                closest_distance = dist
                closest_distance_edge = edge
                closest_distance_point = P

        A = node_pos(graph_copy, closest_distance_edge[0])
        B = node_pos(graph_copy, closest_distance_edge[1])

        if closest_distance > robot_step_size:
            graph_copy.add_node(
                robot_node_id,
                pos_x=round(robot_position[0], 3),
                pos_y=round(robot_position[1], 3),
            )

            if (closest_distance_point == A).all():
                graph_copy.add_edge(robot_node_id, closest_distance_edge[0])
            elif (closest_distance_point == B).all():
                graph_copy.add_edge(robot_node_id, closest_distance_edge[1])
            else:
                graph_copy.add_node(
                    intermediary_node_id,
                    pos_x=round(closest_distance_point[0], 3),
                    pos_y=round(closest_distance_point[1], 3),
                )
                graph_copy.add_edge(intermediary_node_id, closest_distance_edge[0])
                graph_copy.add_edge(intermediary_node_id, closest_distance_edge[1])
                graph_copy.add_edge(robot_node_id, intermediary_node_id)
                graph_copy.remove_edge(*closest_distance_edge)
            return graph_copy, robot_node_id

        if (closest_distance_point == A).all():
            robot_node_id = closest_distance_edge[0]
        elif (closest_distance_point == B).all():
            robot_node_id = closest_distance_edge[1]
        else:
            graph_copy.add_node(
                robot_node_id,
                pos_x=round(closest_distance_point[0], 3),
                pos_y=round(closest_distance_point[1], 3),
            )
            graph_copy.add_edge(robot_node_id, closest_distance_edge[0])
            graph_copy.add_edge(robot_node_id, closest_distance_edge[1])
            graph_copy.remove_edge(*closest_distance_edge)
        return graph_copy, robot_node_id


def point_to_segment(
    A: list | tuple, B: list | tuple, P: list | tuple
) -> tuple[tuple, float]:
    A, B, P = np.array(A), np.array(B), np.array(P)
    AB = B - A
    AP = P - A

    AB_squared = np.dot(AB, AB)
    if AB_squared == 0:
        return (A, np.linalg.norm(A - P))

    t = max(0, min(1, np.dot(AP, AB) / AB_squared))
    Q = A + t * AB
    return (Q, np.linalg.norm(Q - P))
