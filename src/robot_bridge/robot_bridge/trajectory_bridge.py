import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import websocket
import json
import time


class TrajectoryBridge(Node):

    def __init__(self):
        super().__init__('trajectory_bridge')

        # Connect to Node.js adapter
        self.ws = websocket.WebSocket()
        self.ws.connect("ws://localhost:9090")
        self.get_logger().info("Connected to Node.js adapter")

        # Create Action Server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/follow_joint_trajectory',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("Received trajectory goal")

        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names

        for point in trajectory.points:
            positions = point.positions

            # Map joints → Dobot cartesian (basic mapping for now)
            cmd = {
                "cmd": "move_cartesian",
                "pose": {
                    "x": positions[0] * 100,
                    "y": positions[1] * 100,
                    "z": positions[2] * 100,
                    "r": 0
                },
                "speed": 50
            }

            self.ws.send(json.dumps(cmd))
            time.sleep(1)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result


def main():
    rclpy.init()
    node = TrajectoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
