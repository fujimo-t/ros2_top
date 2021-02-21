import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, ParameterType

from asciimatics.screen import ManagedScreen
from asciimatics.scene import Scene

from ros2top.view.node_list_view import NodeListView
from ros2top.model.node_list_model import NodeListModel

class Ros2Top(Node):
    def __init__(self):
        super().__init__('ros2top')

        update_rate = self.declare_parameter('update_rate', 1, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Update rate for list by seconds',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        with ManagedScreen() as screen:
            view = NodeListView(screen)
            model = NodeListModel(self)
            view.update_model(model)

            screen.play([Scene([view], -1)])

def main(args=None):
    rclpy.init(args = args);
    top = Ros2Top()

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
