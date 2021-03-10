from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, ParameterType

class ScenesModel:
    """
    Contains data to displays all scenes.
    """
    def __init__(self, node: Node):
        self.node = node

        self.list_update_frames = node.declare_parameter('list_update_frames', 10, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Update list every this frames. So update rate of the list is 20 (asciimatics frame rate) / list_update_frames.',
            integer_range=[IntegerRange(from_value=1, to_value=20, step=1)]
        )).value

        self.selected_node = None
        self.selected_action = None
