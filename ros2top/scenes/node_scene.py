from asciimatics.screen import Screen
from asciimatics.widgets import Frame, MultiColumnListBox, Layout, Widget

from rclpy.node import Node

from ros2top.scenes.base_scene import BaseScene
from ros2top.scenes.scene_names import SceneNames
from ros2top.widgets.node_list import NodeList

class NodeScene(BaseScene):
    """
    A scene displays list of nodes.
    """
    def __init__(self, screen: Screen, node: Node, list_update_frames: int):
        """
        :param screen: The screen will play this scene.
        :param node: The ROS node to use to discover actions. 
        :param list_update_frames: Update action list every this frames.
        """
        super().__init__(SceneNames.NODE, node, screen)
        
        layout = Layout(columns=[1], fill_frame=True)
        self.main_frame.add_layout(layout)
        layout.add_widget(NodeList(
            node=node,
            height=Widget.FILL_FRAME,
            frame_update_count=list_update_frames
        ))
        
        self.main_frame.fix()
