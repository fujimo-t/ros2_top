import rclpy
from rclpy.node import Node
from asciimatics.screen import ManagedScreen

from ros2top.view.node_list_view import NodeListView
from ros2top.model.node_list_model import NodeListModel

class Ros2Top(Node):
    def __init__(self):
        super().__init__('ros2_top')
        with ManagedScreen() as screen:
            node_list_view = NodeListView(screen)
            node_list_model = NodeListModel(self)
            node_list_view.update(node_list_model)
            screen.refresh()

def main(args=None):
    rclpy.init(args = args);
    top = Ros2Top()
    rclpy.spin(top)

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
