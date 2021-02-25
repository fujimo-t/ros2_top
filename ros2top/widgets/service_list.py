from asciimatics.widgets import MultiColumnListBox
from rclpy.node import Node
import ros2service.api

class ServiceList(MultiColumnListBox):
    """
    Display list of service summary information.
    """

    def __init__(self, node: Node, height: int, frame_update_count: int):
        super().__init__(
            height, 
            columns=["<30", "<0"],
            options=None,
            titles=["Name", "Types"])
        
        self._node = node
        self._frame_update_count = frame_update_count

        self._update_options()
    
    @property
    def frame_update_count(self):
        """
        The number of frames before this Widget should be updated.
        """
        return self._frame_update_count
    
    def update(self, frame_no: int):
        self._update_options()
        super().update(frame_no)

    def _update_options(self):
        service_names_and_types = ros2service.api.get_service_names_and_types(
            node=self._node)
        options = []

        for index, (name, types) in enumerate(service_names_and_types):
            type_str = types[0]
            type_count = len(types)
            if (type_count > 1):
                type_str += ' and ' + type_count + ' types'
            options.append(([
                name,
                type_str,
            ], index))

        self.options = options