from asciimatics.widgets import ListBox
from ros2top.models.action_info_model import ActionInfoModel, ActionEndPoint, ActionEndPointType

from enum import auto, Enum
from typing import List, Tuple

class ActionInfoOptionType(Enum):
    LABEL = auto()
    SERVER = auto()
    CLIENT = auto()

class ActionInfoOption():
    option_type: ActionInfoOptionType
    name: str
    index: int

class ActionInfo(ListBox):
    def __init__(self, height: int, model: ActionInfoModel):
        super().__init__(height, options=[], name='ActionInfoWidget')

        if model != None:
            self.set_model(model)
        
    def _model_to_options(self, model: ActionInfoModel) -> List[Tuple(str, ActionINfoOption)]:
        options = []
        self._append_label_option('Servers: ', options)
        self._append_elements_to_options(model.servers, options)
        self._append_label_option('Clients: ', options)
        self._append_elements_to_options(model.clients, options)
    
    def _append_label_option(self, label: str, options: List[Tuple[str, ActionInfoOption]]) -> None:
        options.append((label, ActionInfoOption(ActionInfoOption.LABEL, label, len(options))))
    
    def _append_elements_to_options(self, endpoints: List[ActionEndPoint], 
        options: List[Tuple[str, ActionInfoOption]]) -> None:
        for action_endpoint in endpoints:
            option_type = ActionInfoOptionType.SERVER if action_endpoint.endpoint_type == ActionEndPointType.SERVER 
                else ActionEndPointType.CLIENT
            new_option = ActionInfoOptionType(option_type, action_endpoint.node_name, len(options))
            label = f'  {action_endpoint.node_name} : {','.join(action_endpoint.types)}'
            options.append((action_endpoint.node_name, new_option))

    def set_model(self, model: ActionInfoModel):
        self.options = self._model_to_options(model)