import rclpy.action 
from rclpy.node import Node

import ros2node.api
from ros2node.api import NodeName, TopicInfo

class NodeInfoModel():
    """
    Information of a node.
    """
    def __init__(self, node: Node, name: NodeName):
        """
        :param node: A node used to discover the target node's information.
        :param name: Name of a target node.
        """
        self.name = name
        self.subscribers = ros2node.api.get_subscriber_info(node=node, remote_node_name=name.full_name)
        self.publishers = ros2node.api.get_publisher_info(node=node, remote_node_name=name.full_name)
        self.service_servers = ros2node.api.get_service_server_info(node=node, remote_node_name=name.full_name)
        self.service_clients = ros2node.api.get_service_client_info(node=node, remote_node_name=name.full_name)
        self.action_servers = [
            TopicInfo(x[0], x[1]) for x in rclpy.action.get_action_server_names_and_types_by_node(node, name.name, name.namespace)
        ]
        self.action_clients = [
            TopicInfo(x[0], x[1]) for x in rclpy.action.get_action_client_names_and_types_by_node(node, name.name, name.namespace)
        ]
        
