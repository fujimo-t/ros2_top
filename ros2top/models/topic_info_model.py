from rclpy.node import Node

class TopicInfoModel():
    """
    Information of a topic.
    """
    def __init__(self, node: Node, name: str):
        """
        :param node: A node used to discover the target topic's information.
        :param name: Name of the target topic.
        """
        self.name = name
        self.publishers = node.get_publishers_info_by_topic(name)
        self.subscribers = node.get_subscriptions_info_by_topic(name)