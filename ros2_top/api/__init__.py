
import rclpy.node
import ros2node.api

def count_connected_publish_topics(node, publish_topics):
    """
    Count publish topic which have subscribers
    
    :param node: rclpy.node.Node to call rclpy API
    :param publish_topics: ros2node.api.TopicInfo[]
    :returns: Num of topic in publish_topics which have subscribers
    """
    connected_publish_count = 0
    for topic in publish_topics:
        if (node.count_subscribers(topic[0]) > 0):
            connected_publish_count += 1
    
    return connected_publish_count


def count_subscribe_topics(node, subscribe_topics):
    """
    Count subscribe topic which have publisher
    
    :param node: rclpy.node.Node to call rclpy API
    :param subscribe_topics: ros2node.api.TopicInfo[]
    :returns: Num of topic in subscribe_topics which have publishers
    """
    connected_subscribe_count = 0
    for topic in subscribe_topics:
        if (node.count_publishers(topic[0]) > 0):
            connected_subscribe_count += 1
    
    return connected_subscribe_count

def has_lifecycle(node, node_name):
    """
    Check a node named node_name has lifecycle

    :param node: rclpy.node.Node to call rclpy API
    :param node_name: Name of target node
    :returns: True if it has lifecycle
    """
    node = node.get_service_names_and_types_by_node(node_name.name, node_name.namespace)

