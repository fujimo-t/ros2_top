from threading import Thread

import rclpy
from rclpy.node import Node

from asciimatics.event import Event, KeyboardEvent
from asciimatics.exceptions import NextScene, ResizeScreenError
from asciimatics.screen import Screen
from asciimatics.scene import Scene

from ros2top.models.scenes_model import ScenesModel
from ros2top.scenes.action_scene import ActionScene
from ros2top.scenes.node_scene import NodeScene
from ros2top.scenes.service_scene import ServiceScene
from ros2top.scenes.topic_scene import TopicScene

class Ros2Top():
    def __init__(self):
        node = Node('ros2top')

        # spin on another thread
        thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
        thread.start()

        self.model = ScenesModel(node)

        start_scene = None

        while rclpy.ok():
            try:
                Screen.wrapper(self.screen_main, arguments=[start_scene])
                break
            except ResizeScreenError as e:
                start_scene = e.scene
        
        node.destroy_node()
        rclpy.shutdown()

    def screen_main(self, screen: Screen, start_scene: Scene) -> None:            
        scenes = [
            # If start_scene is None, initial scene is node
            NodeScene(screen, self.model),
            ActionScene(screen, self.model),
            ServiceScene(screen, self.model),
            TopicScene(screen, self.model)
        ]
        screen.play(
            scenes, 
            stop_on_resize=True,
            start_scene=start_scene)
            
def main(args=None):
    rclpy.init(args = args)
    ros2top = Ros2Top()

if __name__ == '__main__':
    main()
