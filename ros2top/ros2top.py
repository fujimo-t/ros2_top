from threading import Thread

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, ParameterType

from asciimatics.screen import ManagedScreen
from asciimatics.scene import Scene

from ros2top.frames.node_list import NodeListFrame

class Ros2Top(Node):
    def __init__(self):
        super().__init__('ros2top')        
        
        frame_rate = self.declare_parameter('frame_rate', 20, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Frame rate[Hz] used to draw and handle input event',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        list_update_frames = self.declare_parameter('list_update_frames', 10, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Update list every this frames. So, update rate of the list is frame_rate / list_update_frames.',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        # spin on another thread
        thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        rate = self.create_rate(frame_rate.value)

        with ManagedScreen() as screen:
            screen.set_scenes([Scene([
                NodeListFrame(self, list_update_frames.value, screen)], -1)])
            while rclpy.ok():
                screen.draw_next_frame()
                rate.sleep()

def main(args=None):
    rclpy.init(args = args);
    top = Ros2Top()

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
