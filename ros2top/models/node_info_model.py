from rclpy.node import Node
import ros2node.api

class NodeInfoModel():
    """
    Information of a node.
    """
    def __init__(self, node: Node, name: str):
        """
        :param node: A node used to discover the target node's information.
        :param name: Name of a target node.
        """
        self.name = name
        self.subscribers = ros2node.api.get_subscriber_info(node=node, remote_node_name=name)
        self.publishers = ros2node.api.get_publisher_info(node=node, remote_node_name=name)
        self.service_servers = ros2node.api.get_service_server_info(node=node, remote_node_name=name)
        self.service_clients = ros2node.api.get_service_client_info(node=node, remote_node_name=name)
        self.action_servers = ros2node.api.get_action_server_info(node=node, remote_node_name=name)
        self.action_clients = ros2node.api.get_action_client_info(node=node, remote_node_name=name)
    