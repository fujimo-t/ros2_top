from threading import Thread

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, ParameterType

from asciimatics.screen import ManagedScreen
from asciimatics.scene import Scene

from ros2top.frames.node_frame import NodeFrame
from ros2top.scene_list import SceneList

class Ros2Top(Node):
    def __init__(self):
        super().__init__('ros2top')

        list_update_frames = self.declare_parameter('list_update_frames', 10, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Update list every this frames. So, update rate of the list is 20(asciimatics frame rate) / list_update_frames.',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))
        
        initial_scene = self.declare_parameter('initial_scene', "Node", ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Initial scene. Node or Topic or Service or Action.',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        # spin on another thread
        thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        with ManagedScreen() as screen:
            scenes = SceneList.scenes(self, list_update_frames.value, screen)
            screen.play(scenes)
            
def main(args=None):
    rclpy.init(args = args);
    top = Ros2Top()

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
