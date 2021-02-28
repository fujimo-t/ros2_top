from asciimatics.screen import Screen
from asciimatics.widgets import Frame, MultiColumnListBox, Layout, Widget

from rclpy.node import Node

from ros2top.scenes.base_scene import BaseScene
from ros2top.scenes.scene_names import SceneNames
from ros2top.widgets.topic_list import TopicList

class TopicScene(BaseScene):
    def __init__(self, screen: Screen, node: Node, list_update_frames: int):
        """
        :param screen: The screen will play this scene.
        :param node: The ROS node to use to discover actions. 
        :param list_update_frames: Update action list every this frames.
        """
        super().__init__(SceneNames.TOPIC, node, screen)
        
        layout = Layout(columns=[1], fill_frame=True)
        self.frame.add_layout(layout)
        layout.add_widget(TopicList(
            node = node,
            height=Widget.FILL_FRAME,
            frame_update_count=list_update_frames
        ))
        
        self.frame.fix()
