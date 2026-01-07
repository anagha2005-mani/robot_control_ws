#!/usr/bin/env python3
"""
State Manager Node for Dobot Backend (ROS 2 Jazzy)

Responsible for:
- Tracking robot / simulation state
- Monitoring joint states
- Providing readiness information to other backend components

This is a research-stage stub without hardware dependency.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class StateManager(Node):
    def __init__(self):
        super().__init__('dobot_state_manager')

        # Parameters
        self.declare_parameter('simulation_mode', True)

        self.simulation_mode = self.get_parameter(
            'simulation_mode'
        ).get_parameter_value().bool_value

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.ready_publisher = self.create_publisher(
            Bool,
            'backend/ready',
            10
        )

        # Internal state
        self.latest_joint_state = None

        # Timer to publish readiness
        self.timer = self.create_timer(
            1.0,
            self.publish_ready_state
        )

        self.get_logger().info(
            f"State Manager started (simulation_mode={self.simulation_mode})"
        )

    def joint_state_callback(self, msg: JointState):
        """
        Stores the latest joint state.
        """
        self.latest_joint_state = msg

    def publish_ready_state(self):
        """
        Publishes whether the backend is ready.
        For now, readiness is defined as:
        - Joint states have been received at least once
        """
        ready_msg = Bool()

        ready_msg.data = self.latest_joint_state is not None

        self.ready_publisher.publish(ready_msg)


def main(args=None):
    rclpy.init(args=args)

    node = StateManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()