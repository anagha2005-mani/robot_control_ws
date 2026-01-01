#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from pydobot import Dobot
import time


class MoveItToDobot(Node):
    def __init__(self):
        super().__init__('moveit_to_dobot')

        self.get_logger().info("Connecting to Dobot...")
        self.dobot = Dobot(port="/dev/ttyACM0", verbose=False)

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.get_logger().info("MoveIt → Dobot bridge READY")

    def trajectory_callback(self, msg):
        if not msg.points:
            return

        # Take the FINAL point of the trajectory
        final_point = msg.points[-1]
        joints = final_point.positions

        # Simple joint → cartesian mapping (TEMP, SAFE)
        j1, j2, j3, j4 = joints

        self.get_logger().info(
            f"Executing MoveIt goal joints: {joints}"
        )

        # VERY IMPORTANT: slow & safe motion
        self.dobot.move_to(
            200,           # X (keep fixed initially)
            0,             # Y
            100 + j2 * 50, # Z (mapped from joint2)
            j1 * 57.3      # R (rad → deg)
        )

        time.sleep(0.5)


def main():
    rclpy.init()
    node = MoveItToDobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
