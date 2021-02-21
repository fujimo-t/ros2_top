import rclpy
from rclpy.node import Node
from asciimatics.screen import ManagedScreen
from asciimatics.scene import Scene

from ros2top.view.node_list_view import NodeListView
from ros2top.model.node_list_model import NodeListModel

import pprint

class Ros2Top(Node):
    def __init__(self):
        super().__init__('ros2_top')
        with ManagedScreen() as screen:
            view = NodeListView(screen)
            model = NodeListModel(self)
            view.update_model(model)

            self.get_logger().info(pprint.pformat(model.node_list))

            screen.play([Scene([view], -1)])

def main(args=None):
    rclpy.init(args = args);
    top = Ros2Top()
    rclpy.spin(top)

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
