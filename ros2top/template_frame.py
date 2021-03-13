from asciimatics.exceptions import NextScene
from asciimatics.screen import Screen
from asciimatics.widgets import Button, Frame, Label, Layout

from ros2top.scenes.scene_name import SceneName

from typing import Callable, List, NamedTuple, NoReturn, Tuple

class SceneCommand(NamedTuple):
    """
    Command with shortcut key displayed on top or bottom command list.
    """
    button_label: str
    key_name: str
    key_code: int
    callback: Callable

class TemplateFrame(Frame):
    """
    Template for a frame used inside of each scene.

    This has the top layout for list switcher.
    """
    def __init__(self, screen: Screen):
        super().__init__(screen, screen.height, screen.width, has_border=False, can_scroll=False)
        self._add_top_layout()

    def _add_top_layout(self) -> None:
        """
        Add the top frame with list switcher.
        """
        list_scenes = [
            SceneName.NODE, 
            SceneName.TOPIC,
            SceneName.SERVICE,
            SceneName.ACTION
        ]
        # Closure for callback of top commands
        def raise_next_to(scene_name: SceneName):
            def raise_next() -> NoReturn:
                raise NextScene(scene_name.value)
            return raise_next

        top_commands = [
            SceneCommand(SceneName.NODE.value, 'F1', Screen.KEY_F1, self._raise_next_to(SceneName.NODE))
            SceneCommand(SceneName.TOPIC.value, 'F2', Screen.KEY_F2, self._raise_next_to(SceneName.TOPIC))
            SceneCommand(SceneName.SERVICE.value, 'F3', Screen.KEY_F3, self._raise_next_to(SceneName.SERVICE))
            SceneCommand(SceneName.ACTION.value, 'F4', Screen.KEY_F4, self._raise_next_to(SceneName.ACTION))
        ]

        top_layout = Layout([1] * (1 + len(list_scenes)))
        self.add_layout(top_layout)

        top_layout.add_widget(Label('Switch to list:'))

        for column, scene in enumerate(list_scenes, 1):
            def raise_next_to(scene_name: SceneName):
                def raise_next() -> NoReturn:
                    raise NextScene(scene_name.value)
                return raise_next

            top_layout.add_widget(Button(scene.value, raise_next_to(scene)), column)
    
    def _raise_next_to(self, scene_name: SceneName) -> Callable:
        """
        Return closure to switch to specified scene.
        """
        def raise_next() -> NoReturn:
            raise NextScene(scene_name.value)
        return raise_next
    
    def add_commands_layout(self, label: str, commands: List[SceneCommand]):
        """
        Add a layout has list of commands displayed as buttons.
        """
        layout = Layout([1] * (1 + len(commands)))
        self.add_layout(layout)

        layout.add_widget(Label(str))

        for column, command in enumerate(commands, 1):
            bottom_layout.add_widget(Button(command.button_label, command.callback), column)

    def add_bottom_layout(self, commands: List[Tuple[str, Callable]]):
        """
        Add a layout has list of commands displayed as buttons.
        """
        bottom_layout = Layout([1] * (1 + len(commands)))
        self.add_layout(bottom_layout)

        bottom_layout.add_widget(Label('Commands:'))

        for column, (label, callback) in enumerate(commands, 1):
            bottom_layout.add_widget(Button(label, callback), column)