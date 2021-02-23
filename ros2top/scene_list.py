from asciimatics.scene import Scene
from asciimatics.screen import Screen

from collections import namedtuple
from typing import Sized

from rclpy.node import Node
from ros2top.frames.node_frame import NodeFrame


SceneInfo = namedtuple('SceneInfo', ['name', 'frame_class'])

class SceneList:
    """
    List of scenes. Each scene displays a tab e.g. Node List Tab
    """

    # Tabs are displayed by this order
    scene_info_list = [
        SceneInfo('Node', NodeFrame),
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
