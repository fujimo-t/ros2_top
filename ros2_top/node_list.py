import rclpy
import ros2node.api

class NodeList:
    def __init__(self, node):
        self._node = node
            self._node_list = []

        for node_name in get_node_names(node):
            node_info = new NodeInfo(node, node_name)
            self._node_list.append(node_info)

class NodeInfo:
    def __init__(self, node, node_name):
        self._node = node
        self.name = node_name

        self.count_publish_topics()
        self.count_subscribe_topics()

    def count_publish_topics(self):
        publish_topics = self._node.get_publisher_names_and_types_by_node(
            self.name, self.namespace)
        self.publish_count = len(publish_topics)

        # Number of publish topic which has subscribers
        self.connected_publish_count = 0
        for topic in publish_topics:
            if (self._node.count_subscribers(topic[0]) > 0):
                self.connected_publish_count++;

    def count_subscribe_topics(self):
        subscribe_topics = self._node.get_subscriptions_info_by_topic(
            self.name, self.namespace)
        self.subscribe_count = len(subscribe_topics)

        # Number of publish topic which has subscribers
        self.connected_subscribe_count = 0
        for topic in subscribe_topics:
            if (self._node.count_publishers(topic[0]) > 0):
                self.connected_subscribe_count++;

