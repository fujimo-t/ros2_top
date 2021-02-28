from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.widgets import Frame, Label, Layout

from ros2top.scenes.scene_names import SceneNames
from ros2top.shortcuts import SCENE_SHORTCUT_LIST

from abc import ABC
from rclpy.node import Node

class BaseScene(Scene, ABC):
    """
    Base class for scenes.

    This contains global shortcut labels to top 
    """
    def __init__(self, name: SceneNames, node: Node, screen: Screen):
        """
        :param name: Used as Scene name and frame title inside of this scene.
        :param screen: The screen will play this scene.
        :param node: The ROS node to use to discover actions. 
        """
        super().__init__([], -1, name=name.value)

        self.node = node
        self.frame = Frame(screen, screen.height * 2, screen.width, name=name.value+'Frame', title=name.value, x=0, y=0)
        self.add_effect(self.frame)

        # Add global shortcut list to top
        shortcuts_layout = Layout([1])
        self.frame.add_layout(shortcuts_layout)
        shortcuts_label = 'Switch list to: '
        for shortcut in SCENE_SHORTCUT_LIST:
            shortcuts_label += '{}[{}] '.format(shortcut.scene_name.value, shortcut.key_name)
        shortcuts_layout.add_widget(Label(shortcuts_label))
