import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory

class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge')

        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',   # ✅ confirmed correct topic
            self.callback,
            10
        )

        self.get_logger().info(
            'Trajectory bridge started. Waiting for MoveIt planned trajectories...'
        )

    def callback(self, msg):
        if not msg.trajectory:
            return

        robot_traj = msg.trajectory[0].joint_trajectory

        if not robot_traj.points:
            return

        final_point = robot_traj.points[-1]
        joint_positions = final_point.positions

        self.get_logger().info(
            f'Planned joint angles (rad): {joint_positions}'
        )

        # 🔽 NEXT STEP: send these angles to Dobot hardware
        # self.send_to_dobot(joint_positions)

def main():
    rclpy.init()
    node = TrajectoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
