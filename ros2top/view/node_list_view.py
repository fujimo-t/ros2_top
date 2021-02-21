from asciimatics.screen import Screen
from asciimatics.widgets import Frame, MultiColumnListBox, Layout

from ros2top.model.node_list_model import NodeListModel

class NodeListView(Frame):
    def __init__(self, screen: Screen):
        super(NodeListView, self).__init__(
            screen,
            screen.width, 
            screen.height,
            title = "Node List")

        layout = Layout(columns=[1], fill_flame=True)
        self._list_box = MultiColumnListBox(
            height=self.height,
            columns=["<0", "21<", "7>", "7>", "7>", "7>"],
            options=None,
            titles=["Name", "Lifecycle state", "Pubs", "(Subd)", "Subs", "(Pubd)"])
        layout.add_widget(self._list_box)
        self.add_layout(layout)
        
        self.fix()

    def update(self):
        model = NodeListModel()
        options = []
        for (index, node_summary) in enumerate(model.node_list):
            name = node_summary.name.full_name
            state_label = node_summary.has_lifecycle 
                ? node_summary.state.state_label
                : "No lifecycle"
            options.append(([
                node_summary.name.full_name,
                state_label,
                node_summary.publish_topic_count,
                node_summary.connected_publish_topic_count,
                node_summary.subscribe_topic_count,
                node_summary.connected_subscribe_topic_count
            ], index)
        
        self._list_box.options = options
        