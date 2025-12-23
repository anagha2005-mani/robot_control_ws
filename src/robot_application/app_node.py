import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .ros_api import RosAPI
from .command_parser import CommandParser
from .state_manager import StateManager


class ApplicationNode(Node):
    def __init__(self):
        super().__init__('robot_application')

        self.ros_api = RosAPI(self)
        self.parser = CommandParser()
        self.state = StateManager()

        # Incoming commands from UI
        self.create_subscription(
            String,
            '/ui_command',
            self.ui_callback,
            10
        )

        self.get_logger().info("Application Layer ready")

    def ui_callback(self, msg: String):
        # TEMP: assume JSON-like string later
        command = {"type": msg.data}
        parsed = self.parser.parse(command)
        self.ros_api.send_command(parsed)


def main():
    rclpy.init()
    node = ApplicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

