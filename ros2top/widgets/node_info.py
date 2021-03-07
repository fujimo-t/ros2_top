from asciimatics.parsers import AsciimaticsParser
from asciimatics.widgets import ListBox

from ros2node.api import TopicInfo
from ros2top.models.node_info_model import NodeInfoModel

from enum import auto, Enum
from typing import Callable, Tuple

class NodeInfoOptionType(Enum):
    LABEL: auto()
    ACTION: auto()
    SERVICE: auto()
    TOPIC: auto()

class NodeInfoOption(NamedTuple):
    type: NodeInfoOptionType
    name: str

class NodeInfo(ListBox):
    """
    Display a node info.

    TODO: Link to other info scene by select topic, service, action names.
    """
    def __init__(self, height: int, model: NodeInfoModel, on_change: Callable):
        super().__init__(height, options=[], name='NodeInfoWidget', parser=AsciimaticsParser(),
                         on_change=on_change)
        
        if model != None:
            self.set_model(model)

    def _model_to_option(self, model: NodeInfoModel) -> list[Tuple[str, NodeInfoOption]]:
        options = []

        # Add topics to options
        options.append(('Subscribers:', NodeInfoOption(NodeInfoOptionType.LABEL, ''))
        self._append_topics_to_options(model.subscribers, NodeInfoOptionType.TOPIC, options)
        options.append(('Publishers:', NodeInfoOption(NodeInfoOptionType.LABEL, ''))
        self._append_topics_to_options(model.publishers, NodeInfoOptionType.TOPIC, options)
        # Add services to options
        options.append(('Service Servers:', NodeInfoOption(NodeInfoOptionType.LABEL, ''))
        self._append_topics_to_options(model.service_servers, NodeInfoOptionType.SERVICE, options)
        options.append(('Service Clients:', NodeInfoOption(NodeInfoOptionType.LABEL, ''))
        self._append_topics_to_options(model.service_clients, NodeInfoOptionType.SERVICE, options)
        # Add actions to options
        options.append(('Action Servers:', NodeInfoOption(NodeInfoOptionType.LABEL, ''))
        self._append_topics_to_options(model.action_servers, NodeInfoOptionType.ACTION, options)
        options.append(('Action Clients:', NodeInfoOption(NodeInfoOptionType.LABEL, ''))
        self._append_topics_to_options(model.action_clients, NodeInfoOptionType.ACTION, options)

        return options
    
    def _append_topics_to_options(self, topics: list[TopicInfo], type: NodeInfoOptionType,
                                  options: list[Tuple[str, NodeInfoOption]]) -> None:
        for topic in topics:
            (fg, attr, bg) = self._pick_colours('button')
            label = '  ' + f'${{{fg},{attr},{bg}}}' + topic.name + ', '.join(topic.types)
            options.append((label, NodeInfoOption(type, topic.name)))

    def set_model(self, model: NodeInfoModel):
        self.options = self._model_to_option(model)