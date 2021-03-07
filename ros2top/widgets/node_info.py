from asciimatics.widgets import ListBox

from ros2node.api import TopicInfo
from ros2top.models.node_info_model import NodeInfoModel

from enum import auto, Enum
from typing import Callable, List, NamedTuple, Tuple

class NodeInfoOptionType(Enum):
    LABEL = auto()
    ACTION_CLIENT = auto()
    ACTION_SERVER = auto()
    SERVICE_SERVER = auto()
    SERVICE_CLIENT = auto()
    PUBLISHER = auto()
    SUBSCRIBER = auto()

class NodeInfoOption(NamedTuple):
    option_type: NodeInfoOptionType
    name: str

class NodeInfo(ListBox):
    """
    Display a node info.

    TODO: Link to other info scene by select topic, service, action names.
    """
    def __init__(self, height: int, model: NodeInfoModel=None, on_change: Callable=None):
        super().__init__(height, options=[], name='NodeInfoWidget', on_change=on_change)
        
        if model != None:
            self.set_model(model)

    def _model_to_option(self, model: NodeInfoModel) -> List[Tuple[str, NodeInfoOption]]:
        options = []

        # Add topics to options
        self._append_label_option('Subscribers:', options)
        self._append_topics_to_options(model.subscribers, NodeInfoOptionType.SUBSCRIBER, options)
        self._append_label_option('Publishers:', options)
        self._append_topics_to_options(model.publishers, NodeInfoOptionType.PUBLISHER, options)
        # Add services to options
        self._append_label_option('Service servers:', options)
        self._append_topics_to_options(model.service_servers, NodeInfoOptionType.SERVICE_SERVER, options)
        self._append_label_option('Service clients:', options)
        self._append_topics_to_options(model.service_clients, NodeInfoOptionType.SERVICE_CLIENT, options)
        # Add actions to options
        self._append_label_option('Action servers:', options)
        self._append_topics_to_options(model.action_servers, NodeInfoOptionType.ACTION_SERVER, options)
        self._append_label_option('Action clients:', options)
        self._append_topics_to_options(model.action_clients, NodeInfoOptionType.ACTION_CLIENT, options)

        return options
    
    def _append_label_option(self, label: str, options: List[Tuple[str, NodeInfoOption]]) -> None:
        options.append((label, NodeInfoOption(NodeInfoOptionType.LABEL, label)))
    
    def _append_topics_to_options(self, topics: List[TopicInfo], option_type: NodeInfoOptionType,
                                  options: List[Tuple[str, NodeInfoOption]]) -> None:
        for topic in topics:
            label = '  ' + topic.name + ' : ' + ', '.join(topic.types)
            options.append((label, NodeInfoOption(option_type, topic.name)))

    def set_model(self, model: NodeInfoModel):
        self.options = self._model_to_option(model)