import rclpy.action
from rclpy.node import Node

import ros2node.api

from enum import auto, Enum
from typing import List, NamedTuple

class ActionEndPointType(Enum):
    SERVER = auto()
    CLIENT = auto()

class ActionEndPoint(NamedTuple):
    node_name: str
    action_types: List[str]
    endpoint_type: ActionEndPointType

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
            clients_in_node = rclpy.action.get_action_client_names_and_types_by_node(
                node, node_name.name, node_name.namespace)
            self.clients.extend([ActionEndPoint(node_name.full_name, types, ActionEndPointType.CLIENT) 
                for (action_name, types) in clients_in_node if action_name == self.name
            ])

            servers_in_node = rclpy.action.get_action_server_names_and_types_by_node(
                node, node_name.name, node_name.namespace)
            self.servers.extend([ActionEndPoint(node_name.full_name, types, ActionEndPointType.SERVER)
                for (action_name, types) in servers_in_node if action_name == self.name
            ])
