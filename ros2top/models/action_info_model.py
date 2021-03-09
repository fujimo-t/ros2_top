from rclpy.node import Node
import rclpy.action
import ros2node.api

class ActionInfoModel():
    """
    Information of a action.
    """
    def __init__(self, node: Node, name: str):
        self.name = name
        self.clients = []
        self.servers = []

        node_names = ros2node.api.get_node_names(node=node)
        for node_name in node_names:
            clients = rclpy.action.get_action_client_names_and_types_by_node(
                node, node_name.name, node_name.namespace)
            self.clients.extend([
                (node_name, client[1]) for client in clients if client[0] == self.name
            ])

            servers = rclpy.action.get_action_server_names_and_types_by_node(
                node, node_name.name, node_name.namespace)
            self.servers.extend([
                (node_name, server[1]) for server in servers if server[0] == self.name
            ])
