from rclpy.node import Node
import ros2lifecycle.api
import ros2node.api
from lifecycle_msgs.msg import State

class NodeListModel:
    def __init__(self, node: Node):
        self._node = node
        self.node_list = []

        for node_name in ros2node.api.get_node_names(node=node):
            node_info = NodeSummaryModel(node, node_name)
            self.node_list.append(node_info)

class NodeSummaryModel:
    """
    A node's summary information which displayed on node list view as a row.
    """
    def __init__(self, node: Node, node_name: ros2node.api.NodeName):
        self.name = node_name

        # Count publish topics
        publish_topics = ros2node.api.get_publisher_info(node=node, remote_node_name=self.name.full_name)
        self.publish_topic_count = len(publish_topics)
        self.connected_publish_topic_count = len([
            publish_topic for publish_topic in publish_topics 
            if node.count_subscribers(publish_topic.name) > 0])

        # Count subscribe topics
        subscribe_topics = ros2node.api.get_subscriber_info(node=node, remote_node_name=self.name.full_name)
        self.subscribe_topic_count = len(subscribe_topics)
        self.connected_subscribe_topic_count = len([
            subscribe_topic for subscribe_topic in subscribe_topics 
            if node.count_publishers(subscribe_topic.name) > 0])

        # Check lifecycle
        services = node.get_service_names_and_types_by_node(self.name.name, self.name.namespace)
        self.has_lifecycle = ros2lifecycle.api._has_lifecycle(self.name.full_name, services)

        if (self.has_lifecycle):
            state = ros2lifecycle.api.call_get_states(node, self.name.full_name)
            self.state = state[self.name.full_name]
