#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pydobot import Dobot
import math

class DobotJointStatePublisher(Node):
    def __init__(self):
        super().__init__('dobot_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        try:
            self.get_logger().info("Connecting to Dobot...")
            self.dobot = Dobot(port="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5432035886-if00",verbose=False)

            self.get_logger().info("Dobot connected successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Dobot: {e}")
            raise
        
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        self.get_logger().info("Dobot → ROS joint_states bridge started")
    
    def publish_joint_states(self):
        try:
            # pose() returns a tuple
            x, y, z, r, j1, j2, j3, j4 = self.dobot.pose()
            
            # Convert degrees → radians and apply offsets
            joint_positions = [
                math.radians(j1),          # base
                math.radians(j2 - 20.0),   # shoulder offset
                math.radians(j3 - 7.0),    # elbow offset
                math.radians(j4),          # wrist
            ]
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [
                'magician_joint_1',
                'magician_joint_2',
                'magician_joint_3',
                'magician_joint_4',
            ]
            msg.position = joint_positions
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error reading/publishing joint states: {e}")

def main():
    rclpy.init()
    try:
        node = DobotJointStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()