from asciimatics.frame import Frame
from asciimatics.widget import MultiColumnListBox

class NodeListView(Frame):
    def __init__(self, screen, model):
        super(NodeListView, self).__init__(screen,
                                           screen.width, 
                                           screen.height,
                                           title = "Node List")
        self._model = model

        self._list_box = MultiColumnListBox(2, 
                                            ["1"], 
                                            [("aa", 1), ("bb", 2)]
                                            titles=["Name"])
