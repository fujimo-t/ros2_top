import rclpy
import ros2node.api

import ros2_top.api

class NodeList:
    def __init__(self, node):
        self._node = node
        self._node_list = []

        for node_name in get_node_names(node):
            node_info = NodeSummary(node, node_name)
            self._node_list.append(node_info)

class NodeSummary:
    """
    A node's summary information which displayed on node list view as a row.
    """
    def __init__(self, node, node_name):
        self.name = node_name

        publish_topics = get_publisher_info(node, node_name.full_name)
        self.publish_topic_count =  len(publish_topics)
        self.connected_publish_topic_count = count_connected_publish_topics(node, publish_topics)

        subscribe_topics = get_subscriber_info(node, node_name.full_name)
        self.subscribe_topic_count = len(subscribe_topics)
        self.connected_subscribe_topic_count = count_connected_subscribe_topics(node, subscribe_topics)

    

