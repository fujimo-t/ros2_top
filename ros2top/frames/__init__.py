from asciimatics.widgets import Frame, Label, Layout
from enum import Enum
from ros2top.frames.node_frame import NodeFrame
from ros2top.scene_list import SceneList

def add_tab_indicator_layout(frame: Frame):
    """
    Add a layout which contains top labels to frame
    """
    columns = [1] * SceneList.count()
    layout = Layout(columns)
    frame.add_layout(layout)

    for i, scene_info in enumerate(SceneList.scene_info_list):
        label = '{}[{}]'.format(scene_info.name, scene_info.shortcut.name)
        layout.add_widget(Label(label), i)
