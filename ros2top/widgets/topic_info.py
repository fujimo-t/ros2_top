from asciimatics.widgets import ListBox
from rclpy.topic_endpoint_info import TopicEndpointInfo
from ros2top.models.topic_info_model import TopicInfoModel
from typing import List, NamedTuple, Tuple

class TopicInfoOptionType:
    LABEL = auto()
    PUBLISHER = auto()
    SUBSCRIBER = auto()

class TopicInfoOption(NamedTuple):
    option_type: TopicInfoOptionType
    name: str
    index: int

class TopicInfo(ListBox):
    """
    Display a topic info.
    """
    def __init__(self, height: int, model: TopicInfoModel):
        super().__init__(height, options=[], name='TopicInfoWidget')

        if model != None:
            self.set_model(model)

    def _model_to_options(self, model: TopicInfoModel) -> List[Tuple[str, TopicInfoOption]]:
        options = []
        self._append_label_option("Subscribers:", options)
        self._append_topics_to_options(model.subscribers, TopicInfoOptionType.SUBSCRIBER)
        self._append_label_option("Publishers:", options)
        self._append_topics_to_options(model.publishers, TopicInfoOptionType.PUBLISHER)
        return options
    
    def _append_label_option(self, label: str, options: List[Tuple[str, TopicInfoOption]]) -> None:
        options.append((label, TopicInfoOption(TopicInfoOptionType.LABEL, label, len(options))))
    
    def _append_topics_to_options(self, topics: List[TopicEndpointInfo], option_type: TopicInfoOptionType
        options: List[Tuple[str, TopicInfoOption]]) -> None:
        for topic in topics:
            label = f'  {topic.node_namespace}{topic.node_name} : {topic.topic_type}'
            option_value = TopicInfoOption(
                option_type, 
                topic.node_namespace + topic.node_name, 
                length(options)
            )
            options.append((label, option_value))

    def set_model(self, model: TopicInfoModel):
        self.options = self._model_to_options(model)