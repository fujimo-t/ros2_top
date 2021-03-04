from asciimatics.exceptions import NextScene
from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.widgets import Button, Frame, Label, Layout

from ros2top.scenes.scene_names import SceneNames

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
        self.frame = Frame(screen, screen.height, screen.width, name=name.value+'Frame', title=name.value, x=0, y=0)
        self.add_effect(self.frame)

        list_scenes = [
            SceneNames.NODE, 
            SceneNames.TOPIC,
            SceneNames.SERVICE,
            SceneNames.ACTION
        ]

        # Add list switcher to top
        top_layout = Layout([1] * (len(list_scenes) + 1))
        self.frame.add_layout(top_layout)
        top_layout.add_widget(Label('Switch list to:'), column=0)
        for column, scene in enumerate(list_scenes, 1):
            
            # Closure
            def raise_next_to(scene_name: SceneNames):
                def raise_next():
                    raise NextScene(scene_name.value)
                return raise_next

            top_layout.add_widget(Button(
                scene.value,
                raise_next_to(scene)
            ), column)

