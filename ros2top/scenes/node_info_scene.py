from ros2top.models.node_info_model import NodeInfoModel
from ros2top.models.scenes_model import ScenesModel
from ros2top.scenes.base_scene import BaseScene
from ros2top.scenes.scene_name import SceneName
from ros2top.widgets.node_info import NodeInfo

from asciimatics.screen import Screen
from asciimatics.widgets import Label, Layout

class NodeInfoScene(BaseScene):
    """
    A scene displays a node information.
    """
    def __init__(self, screen: Screen, model: ScenesModel):
        """
        :param screen: The screen will play this scene.
        :param model: All data to display scenes. 
        """
        super().__init__(SceneName.NODE_INFO, model, screen)

        self.frame._on_load = self.load_node_info

        layout = Layout(columns=[1], fill_frame=True)
        self.frame.add_layout(layout)
        self._name_label = Label('No node selected', align='<')
        layout.add_widget(self._name_label)
        self._info_widget = NodeInfo(screen.height - 2)
        layout.add_widget(self._info_widget)

        self.frame.fix()
    
    def load_node_info(self):
        selected_node = self.model.selected_node 
        if selected_node != None:
            self._name_label.text = 'Node info: ' + selected_node.full_name
            model = NodeInfoModel(self.model.node, selected_node)
            self._info_widget.set_model(model)

