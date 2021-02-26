from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.exceptions import NextScene

from collections import namedtuple
from typing import Sized

from rclpy.node import Node
from ros2top.frames.action_frame import ActionFrame
from ros2top.frames.node_frame import NodeFrame
from ros2top.frames.service_frame import ServiceFrame
from ros2top.frames.topic_frame import TopicFrame


SceneInfo = namedtuple('SceneInfo', ['name', 'frame_class'])

class SceneList:
    """
    List of scenes. Each scene displays a tab e.g. Node List Tab
    """

    # Tabs are displayed by this order
    scene_info_list = [
        SceneInfo('Node', NodeFrame),
        SceneInfo('Topic', TopicFrame),
        SceneInfo('Service', ServiceFrame)
        SceneInfo('Action', ActionFrame)
    ]

    @classmethod
    def count(cls) -> Sized:
        return len(cls.scene_info_list)

    @classmethod
    def names(cls) -> list:
        return [scene_info.name for scene_info in cls.scene_info_list]
    
    @classmethod
    def scenes(cls, node: Node, frame_update_count: int, screen: Screen) -> list:
        scenes = []
        for scene_info in cls.scene_info_list:
            scene = Scene([scene_info.frame_class(node, frame_update_count, screen)], -1)
            scenes.append(scene)
        
        return scenes

    @classmethod
    def next_scene(cls, screen: Screen) -> None:
        raise NextScene()
