from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class RosAPI:
    def __init__(self, node: Node):
        self.node = node

        # Publish commands to controller layer
        self.command_pub = node.create_publisher(
            String,
            '/controller_command',
            10
        )

        # Subscribe to robot joint states
        self.joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.latest_joint_state = None

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

