
from asciimatics.screen import Screen

from ros2top.scenes.scene_names import SceneNames
from typing import NamedTuple

class SceneShortcut(NamedTuple):
    scene_name: SceneNames
    key_name: str
    key_code: int

# Scene shortcut list is displayed top of each scene order by this list
SCENE_SHORTCUT_LIST = [
    SceneShortcut(SceneNames.NODE, 'F1', Screen.KEY_F1),
    SceneShortcut(SceneNames.TOPIC, 'F2', Screen.KEY_F2),
    SceneShortcut(SceneNames.SERVICE, 'F3', Screen.KEY_F3),
    SceneShortcut(SceneNames.ACTION, 'F4', Screen.KEY_F4)
]
