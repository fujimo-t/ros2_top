from asciimatics.widgets import MultiColumnListBox
from rclpy.node import Node
import ros2action.api
import rclpy.action.graph

class ActionList(MultiColumnListBox):
    def __init__(self, node: Node, height: int, frame_update_count: int):
        super().__init__(
            height, 
            columns=["<50%", "<0"],
            options=None,
            titles=["Action name", "Types"])
        
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
        action_names_and_types = rclpy.action.graph.get_action_names_and_types(node=self._node)
        options = []

        for name, types in action_names_and_types:
            options.append(([
                name,
                ','.join(type),
            ], name))

        self.options = options