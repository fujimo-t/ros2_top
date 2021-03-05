from asciimatics.exceptions import NextScene
from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.widgets import Button, Frame, Label, Layout

from ros2top.models.scenes_model import ScenesModel
from ros2top.scenes.scene_name import SceneName
from ros2top.template_frame import TemplateFrame

from abc import ABC

class BaseScene(Scene, ABC):
    """
    Base class for scenes.
    """
    def __init__(self, name: SceneName, model: ScenesModel, screen: Screen):
        """
        :param name: Used as Scene name and frame title inside of this scene.
        :param model: All data to display scenes.
        :param screen: The screen will play this scene.
        """
        super().__init__([], -1, name=name.value)
        self.model = model
        self.frame = TemplateFrame(screen)
        self.add_effect(self.frame)
