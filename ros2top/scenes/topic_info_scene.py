from asciimatics.screen import Screen
from asciimatics.widgets import Label, Layout

from ros2top.models.scenes_model import ScenesModel
from ros2top.models.topic_info_model import TopicInfoModel
from ros2top.scenes.base_scene import BaseScene
from ros2top.scenes.scene_name import SceneName
from ros2top.widgets.topic_info import TopicInfo

class TopicInfoScene(BaseScene):
    """
    A scene displays a topic information.
    """
    def __init__(self, screen: Screen, model: ScenesModel):
        """
        :param screen: The screen will play this scene.
        :param model: All data to display scenes. 
        """
        super().__init__(SceneName.TOPIC_INFO, model, screen)

        self.frame._on_load = self.load_topic_info

        layout = Layout(columns=[1], fill_frame=True)
        self.frame.add_layout(layout)
        self._name_label = Label('No topic selected', align='<')
        layout.add_widget(self._name_label)
        self._info_widget = TopicInfo(screen.height - 2)
        layout.add_widget(self._info_widget)

        self.frame.fix()
    
    def load_topic_info(self):
        selected_topic = self.model.selected_topic 
        if selected_topic != None:
            self._name_label.text = 'Topic info: ' + selected_topic
            model = TopicInfoModel(self.model.node, selected_topic)
            self._info_widget.set_model(model)