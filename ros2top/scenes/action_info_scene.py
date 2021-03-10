from asciimatics.screen import Screen
from asciimatics.widgets import Label, Layout

from ros2top.models.scenes_model import ScenesModel
from ros2top.scenes.base_scene import BaseScene
from ros2top.scenes.scene_name import SceneName
from ros2top.widgets.action_info import ActionInfo

class ActionInfoScene(BaseScene):
    """
    A scene displays a action information.
    """
    def __init__(self, screen: Screen, model: ScenesModel):
        """
        :param screen: The screen will play this scene.
        :param model: All data to display scenes. 
        """
        super().__init__(SceneName.ACTION_INFO, model, screen)

        self.frame._on_load = self.load_action_info

        layout = Layout(columns=[1], fill_frame=True)
        self.frame.add_layout(layout)
        self._name_label = Label('No Action selected', align='<')
        layout.add_widget(self._name_label)
        self._info_widget = ActionInfo(screen.height - 2)
        layout.add_widget(self._info_widget)

        self.frame.fix()

    def load_action_info(self):
        selected_action = self.model.selected_action
        if selected_action != None:
            self._name_label