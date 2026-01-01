import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import websocket
import json

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')

        # Connect to Node.js WebSocket adapter
        self.ws = websocket.WebSocket()
        self.ws.connect("ws://localhost:5001")
        self.get_logger().info("Connected to Node.js adapter")

        # Select Dobot
        self.ws.send(json.dumps({"cmd": "select_robot", "robot": "dobot"}))

        # Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(JointTrajectory, 'joint_trajectory', self.trajectory_callback, 10)

        # Timer to publish joint states at 10Hz
        self.create_timer(0.1, self.publish_joint_states)

    def trajectory_callback(self, msg: JointTrajectory):
        """Receive MoveIt trajectory and forward to Node.js"""
        if len(msg.points) == 0:
            return

        point: JointTrajectoryPoint = msg.points[0]

        cmd = {
            "cmd": "move_cartesian",
            "pose": {
                "x": float(point.positions[0]),
                "y": float(point.positions[1]),
                "z": float(point.positions[2]),
                "r": float(point.positions[3])
            },
            "speed": 50
        }

        self.ws.send(json.dumps(cmd))
        self.get_logger().info(f"Sent trajectory point: {cmd}")

    def publish_joint_states(self):
        """Fetch latest robot pose from Node.js adapter"""
        try:
            self.ws.send(json.dumps({"cmd": "get_position"}))
            data = json.loads(self.ws.recv())
        except Exception as e:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["x", "y", "z", "r"]
        msg.position = [
            float(data.get("x", 0.0)),
            float(data.get("y", 0.0)),
            float(data.get("z", 0.0)),
            float(data.get("r", 0.0)),
        ]

        self.joint_pub.publish(msg)

def main():
    rclpy.init()
    node = RobotBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
