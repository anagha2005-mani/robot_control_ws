import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from DobotDllType import *

class DobotTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('dobot_trajectory_bridge')

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.callback,
            10
        )

        self.api = load()
        ConnectDobot(self.api, "", 115200)
        self.get_logger().info("✅ Dobot connected")

    def callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        point = msg.points[-1]
        joints = point.positions

        # Dobot expects degrees
        j1 = joints[0] * 57.2958
        j2 = joints[1] * 57.2958
        j3 = joints[2] * 57.2958
        j4 = joints[3] * 57.2958

        SetPTPCmd(
            self.api,
            PTPMode.PTPMOVJANGLEMode,
            j1, j2, j3, j4,
            1
        )

def main():
    rclpy.init()
    node = DobotTrajectoryBridge()
    rclpy.spin(node)
    rclpy.shutdown()
