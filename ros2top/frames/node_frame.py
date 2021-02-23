from asciimatics.screen import Screen
from asciimatics.widgets import Frame, MultiColumnListBox, Layout, Widget

from rclpy.node import Node
from ros2top.widgets.node_list import NodeList

class NodeFrame(Frame):
    def __init__(self, node: Node, list_update_frames: int, screen: Screen):
        super().__init__(
            screen,
            screen.height, 
            screen.width,
            title = "Node List")

        self._node_list = NodeList(
            node = node,
            height=Widget.FILL_FRAME,
            frame_update_count=list_update_frames)
        
        layout = Layout(columns=[100], fill_frame=True)
        self.add_layout(layout)
        layout.add_widget(self._node_list)
        
        self.fix()
