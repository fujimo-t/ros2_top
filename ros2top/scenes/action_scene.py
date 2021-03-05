from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.widgets import Frame, MultiColumnListBox, Layout, Widget

from rclpy.node import Node

from ros2top.models.scenes_model import ScenesModel
from ros2top.scenes.scene_name import SceneName
from ros2top.scenes.base_scene import BaseScene
from ros2top.widgets.action_list import ActionList

class ActionScene(BaseScene):
    """
    A Scene displays list of actions.
    """
    def __init__(self, screen: Screen, model: ScenesModel):
        """
        :param screen: The screen will play this scene.
        :param model: All data to display scenes. 
        """
        super().__init__(SceneName.ACTION, model, screen)

        layout = Layout(columns=[1], fill_frame=True)
        self.frame.add_layout(layout)
        layout.add_widget(ActionList(
            node=model.node,
            height=Widget.FILL_FRAME,
            frame_update_count=model.list_update_frames
        ))
        
        self.frame.fix()
