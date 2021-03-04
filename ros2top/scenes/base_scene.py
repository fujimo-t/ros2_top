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

        top_frame_height = 3
        bottom_frame_height = 3
        main_frame_height = screen.height - (top_frame_height + bottom_frame_height)

        # Add list switcher to top frame
        y = 0
        top_frame = Frame(screen, top_frame_height, screen.width, name='TopFrame', title='List select', y=y)
        y += top_frame_height
        self.add_effect(top_frame)

        list_scenes = [
            SceneNames.NODE, 
            SceneNames.TOPIC,
            SceneNames.SERVICE,
            SceneNames.ACTION
        ]

        top_layout = Layout([1] * (len(list_scenes)))
        top_frame.add_layout(top_layout)
        for column, scene in enumerate(list_scenes):
            def raise_next_to(scene_name: SceneNames):
                def raise_next():
                    raise NextScene(scene_name.value)
                return raise_next

            top_layout.add_widget(Button(
                scene.value,
                raise_next_to(scene)
            ), column)
        top_frame.fix()

        self.main_frame = Frame(screen, main_frame_height, screen.width, name='MainFrame', title=name.value, y=y)
        self.add_effect(self.main_frame)
        y += main_frame_height
        
        self.bottom_frame = Frame(screen, bottom_frame_height, screen.width, name="BottomFrame", title='Commands', y=y)
        self.add_effect(self.bottom_frame)

