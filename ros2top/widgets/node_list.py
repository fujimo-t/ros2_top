from asciimatics.widgets import MultiColumnListBox
from rclpy.node import Node
from ros2top.models.node_list_model import NodeListModel

class NodeList(MultiColumnListBox):
    """
    Display list of node summary information.
    """

    def __init__(self, node: Node, height: int, frame_update_count: int):
        super().__init__(
            height, 
            columns=["<0", ">21", ">7", ">7", ">7", ">7"],
            options=None,
            titles=["Name", "Lifecycle state", "Pubs", "(Subd)", "Subs", "(Pubd)"])
        
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
        model = NodeListModel(self._node)
        options = []

        for index, node_summary in enumerate(model.node_list):
            name = node_summary.name.full_name
            state_label = node_summary.state.state_label if node_summary.has_lifecycle else "No lifecycle"
            options.append(([
                node_summary.name.full_name,
                state_label,
                str(node_summary.publish_topic_count),
                str(node_summary.connected_publish_topic_count),
                str(node_summary.subscribe_topic_count),
                str(node_summary.connected_subscribe_topic_count)
            ], index))

        self.options = options