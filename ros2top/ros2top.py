from threading import Thread

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, ParameterType

from asciimatics.event import Event, KeyboardEvent
from asciimatics.exceptions import NextScene, ResizeScreenError
from asciimatics.screen import Screen
from asciimatics.scene import Scene

from ros2top.scenes.action_scene import ActionScene
from ros2top.scenes.node_scene import NodeScene
from ros2top.scenes.service_scene import ServiceScene
from ros2top.scenes.topic_scene import TopicScene
from ros2top.shortcuts import SCENE_SHORTCUT_LIST, SceneShortcut

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

        start_scene = None

        while rclpy.ok():
            try:
                Screen.wrapper(self.screen_main, arguments=[start_scene])
                break
            except ResizeScreenError as e:
                start_scene = e.scene

    def screen_main(self, screen: Screen, start_scene: Scene) -> None:            
        scenes = [
            # If start_scene is None, initial scene is node
            NodeScene(screen, self, self._list_update_frames.value),
            ActionScene(screen, self, self._list_update_frames.value),
            ServiceScene(screen, self, self._list_update_frames.value),
            TopicScene(screen, self, self._list_update_frames.value)
        ]
        screen.play(
            scenes, 
            unhandled_input=self.handle_global_shortcuts,
            stop_on_resize=True,
            start_scene=start_scene)
    
    def handle_global_shortcuts(self, event: Event) -> None:
        if isinstance(event, KeyboardEvent):
            keycode = event.key_code
            for scene_shortcut in SCENE_SHORTCUT_LIST:
                if keycode == scene_shortcut.key_code:
                    raise NextScene(scene_shortcut.scene_name.value)
            
def main(args=None):
    rclpy.init(args = args)
    top = Ros2Top()

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
