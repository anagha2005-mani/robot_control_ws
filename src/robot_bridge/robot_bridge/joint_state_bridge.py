import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import websocket
import json
import threading
import time


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')

        # Publisher
        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # WebSocket connection to Node.js adapter
        self.ws = websocket.WebSocket()
        self.ws.connect("ws://localhost:5001")
        self.get_logger().info("Connected to Node.js adapter")

        # Select robot
        self.ws.send(json.dumps({
            "cmd": "select_robot",
            "robot": "dobot"
        }))

        # Timer to publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_state)

    def publish_joint_state(self):
        try:
            # Request position
            self.ws.send(json.dumps({"cmd": "get_position"}))
            response = self.ws.recv()
            data = json.loads(response)

            # Create JointState message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Generic joint names (robot-agnostic)
            msg.name = [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4"
            ]

            # Dobot does not provide true joint angles here
            # We map Cartesian values for now (acceptable at abstraction layer)
            msg.position = [
                float(data.get("x", 0.0)),
                float(data.get("y", 0.0)),
                float(data.get("z", 0.0)),
                float(data.get("r", 0.0))
            ]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to publish joint state: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
