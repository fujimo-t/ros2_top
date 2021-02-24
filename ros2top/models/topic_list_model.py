import ros2topic.api
from rclpy.node import Node
from collections import namedtuple

TopicSummary = namedtuple('TopicSummary', ['name', 'types', 'publisher_count', 'subscriber_count'])

class TopicListModel:
    def __init__(self, node: Node):
        self.list = []
        names_and_types = ros2topic.api.get_topic_names_and_types(node=node)
        
        for (name, types) in names_and_types:
            self.list.append(TopicSummary(
                name,
                types,
                node.count_publishers(name),
                node.count_subscribers(name)
            ))