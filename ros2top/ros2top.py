from threading import Thread

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

        list_update_rate = self.declare_parameter('list_update_rate', 1, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Update rate[Hz] of data in list',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))
        
        frame_rate = self.declare_parameter('frame_rate', 20, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Frame rate[Hz] used to draw and handle input event',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        # spin on another thread
        thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        rate = self.create_rate(frame_rate.value)

        with ManagedScreen() as screen:
            view = NodeListView(screen)
            model = NodeListModel(self)
            view.update_model(model)

            screen.set_scenes([Scene([view], -1)])
            while rclpy.ok():
                screen.draw_next_frame()
                model = NodeListModel(self)
                view.update_model(model)
                rate.sleep()

def main(args=None):
    rclpy.init(args = args);
    top = Ros2Top()

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
