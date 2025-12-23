import rclpy
from rclpy.node import Node
import websocket
import json
import time

class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.ws = None

        # Connect to WebSocket server
        self.connect_to_adapter()

        # Auto-select Dobot
        self.select_robot("dobot")

        # Timer to run every second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Flag to ensure movement is sent once
        self.test_sent = False

    def connect_to_adapter(self):
        try:
            self.ws = websocket.WebSocket()
            self.ws.connect("ws://localhost:9090")
            self.get_logger().info("Connected to Node.js adapter")
        except Exception as e:
            self.get_logger().error(f"WebSocket connection failed: {e}")

    def send(self, obj):
        try:
            self.ws.send(json.dumps(obj))
        except Exception as e:
            self.get_logger().error(f"Failed to send: {e}")

    def select_robot(self, name):
        self.get_logger().info(f"Selecting robot: {name}")

        # Send robot selection
        self.send({
            "cmd": "select_robot",
            "robot": name
        })

        # Read reply
        try:
            msg = self.ws.recv()
            self.get_logger().info(f"Adapter: {msg}")
        except:
            pass

        # Send connect request
        self.send({"cmd": "connect"})

        try:
            msg = self.ws.recv()
            self.get_logger().info(f"Adapter: {msg}")
        except:
            pass

    def timer_callback(self):
        # Run a demo movement ONCE
        if not self.test_sent:
            self.test_sent = True

            self.get_logger().info("Sending HOME command")
            self.send({"cmd": "home"})
            time.sleep(2)

            self.get_logger().info("Moving to X200 Y0 Z150")
            self.send({
                "cmd": "move_cartesian",
                "pose": {"x": 200, "y": 0, "z": 150, "r": 0},
                "speed": 50
            })

            return

        # Otherwise, keep reading position
        self.send({"cmd": "get_position"})
        try:
            msg = self.ws.recv()
            self.get_logger().info(f"Robot Position: {msg}")
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

