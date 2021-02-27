from threading import Thread

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, ParameterType

from asciimatics.exceptions import ResizeScreenError
from asciimatics.screen import Screen
from asciimatics.scene import Scene

from ros2top.frames.node_frame import NodeFrame
from ros2top.scene_list import SceneList

class Ros2Top(Node):
    def __init__(self):
        super().__init__('ros2top')

        self._list_update_frames = self.declare_parameter('list_update_frames', 10, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Update list every this frames. So, update rate of the list is 20(asciimatics frame rate) / list_update_frames.',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]))

        # spin on another thread
        thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        last_scene = None

        while rclpy.ok():
            try:
                Screen.wrapper(self.screen_main, arguments=[last_scene])
                break
            except ResizeScreenError as e:
                last_scene = e.scene

    def screen_main(self, screen: Screen, start_scene: Scene):
        scenes = SceneList.scenes(self, self._list_update_frames.value, screen)
        screen.play(
            scenes, 
            unhandled_input=SceneList.global_shortcuts,
            stop_on_resize=True,
            start_scene=start_scene)
            
def main(args=None):
    rclpy.init(args = args);
    top = Ros2Top()

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
