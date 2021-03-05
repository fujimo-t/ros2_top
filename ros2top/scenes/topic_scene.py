from asciimatics.screen import Screen
from asciimatics.widgets import Frame, MultiColumnListBox, Layout, Widget

from rclpy.node import Node

from ros2top.models.scenes_model import ScenesModel
from ros2top.scenes.base_scene import BaseScene
from ros2top.scenes.scene_name import SceneName
from ros2top.widgets.topic_list import TopicList

class TopicScene(BaseScene):
    def __init__(self, screen: Screen, model: ScenesModel):
        """
        :param screen: The screen will play this scene.
        :param model: All data to display scenes. 
        """
        super().__init__(SceneName.TOPIC, model, screen)
        
        layout = Layout(columns=[1], fill_frame=True)
        self.frame.add_layout(layout)
        layout.add_widget(TopicList(
            node=model.node,
            height=Widget.FILL_FRAME,
            frame_update_count=model.list_update_frames
        ))
        
        self.frame.fix()
