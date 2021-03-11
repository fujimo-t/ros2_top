from asciimatics.exceptions import NextScene
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
        self._list_widget = TopicList(
            node=model.node,
            height=Widget.FILL_FRAME,
            frame_update_count=model.list_update_frames
        )
        layout.add_widget(self._list_widget)
        self.frame.add_bottom_layout([('Info', self._show_info)])
        
        self.frame.fix()

    def _show_info(self):
        self.model.selected_topic = self._list_widget.value
        raise NextScene(SceneName.TOPIC_INFO.value)