from asciimatics.screen import Screen
from asciimatics.widgets import Frame, MultiColumnListBox, Layout, Widget

from rclpy.node import Node

import ros2top.frames
from ros2top.widgets.action_list import ActionList

class ActionFrame(Frame):
    def __init__(self, node: Node, frame_update_count: int, screen: Screen):
        super().__init__(screen, screen.height, screen.width)

        action_list = ActionList(
            node = node,
            height=Widget.FILL_FRAME,
            frame_update_count=frame_update_count)
        
        ros2top.frames.add_tab_indicator_layout(self)

        layout = Layout(columns=[1], fill_frame=True)
        self.add_layout(layout)
        layout.add_widget(action_list)
        
        self.fix()
