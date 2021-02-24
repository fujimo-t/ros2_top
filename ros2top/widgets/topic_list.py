from asciimatics.widgets import MultiColumnListBox
from rclpy.node import Node
from ros2top.models.topic_list_model import TopicListModel

class TopicList(MultiColumnListBox):
    """
    Display list of topic summary information.
    """

    def __init__(self, node: Node, height: int, frame_update_count: int):
        super().__init__(
            height, 
            columns=["<0", ">21", ">7", ">7"],
            options=None,
            titles=["Name", "Types", "Publishers", "Subscribers"])
        
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
        model = TopicListModel(self._node)
        options = []

        for index, topic_summary in enumerate(model.list):
            type_str = topic_summary.types[0]
            type_count = len(topic_summary.types)
            if (type_count > 1):
                type_str += ' and ' + type_count + ' types'
            options.append(([
                topic_summary.name,
                type_srt,
                str(topic_summary.publisher_count),
                str(topic_summary.subscriber_count),
            ], index))

        self.options = options