import rclpy
from rclpy.node import Node
from asciimatics.screen import ManagedScreen

class Ros2Top(Node):
    def __init__(self):
        super().__init__('ros2_top')

        with ManagedScreen() as screen:
            # TODO
            screen.refresh()

def main(args=None):
    rclpy.init(args = args);
    top = Ros2Top()
    rclpy.spin(top)

    top.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
