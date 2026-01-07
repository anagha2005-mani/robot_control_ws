#!/usr/bin/env python3
"""
Dobot Magician Motion Planner - ROS2 JAZZY VERSION
Handles all motion planning and execution using MoveIt2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# JAZZY: Updated MoveIt imports
from moveit_py.core import MoveItPy, RobotState as MoveItRobotState

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Bool

import numpy as np
import threading
import time
from enum import Enum


class RobotStateEnum(Enum):
    """Robot state enumeration"""
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


class DobotMotionPlanner(Node):
    def __init__(self):
        super().__init__('dobot_motion_planner')
        
        self.get_logger().info("Initializing for ROS2 Jazzy")
        
        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Robot state
        self.current_state = RobotStateEnum.IDLE
        self.current_joints = None
        self.current_pose = None
        self.emergency_stop_active = False
        
        # MoveIt setup
        self.declare_parameter('planning_group', 'dobot_arm')
        self.planning_group = self.get_parameter('planning_group').value
        
        try:
            # JAZZY: Pass node instance directly
            self.moveit = MoveItPy(node=self)
            self.robot_model = self.moveit.get_robot_model()
            self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
            self.dobot_arm = self.moveit.get_planning_component(self.planning_group)
            self.get_logger().info(f"MoveIt initialized for group: {self.planning_group}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt: {str(e)}")
            raise
        
        # Predefined poses (adjust these for your specific Dobot setup)
        self.predefined_poses = {
            'home': [0.0, 0.0, 0.0, 0.0],  # All joints at zero
            'ready': [0.0, -0.785, 1.57, 0.0],  # Ready position
            'pick_approach': [0.3, 0.0, 0.2, 0.0],  # XYZ in meters
            'pick': [0.3, 0.0, 0.05, 0.0],
            'place_approach': [-0.3, 0.0, 0.2, 0.0],
            'place': [-0.3, 0.0, 0.05, 0.0],
        }
        
        # Safety limits (adjust for Dobot Magician)
        self.workspace_limits = {
            'x_min': -0.4, 'x_max': 0.4,
            'y_min': -0.4, 'y_max': 0.4,
            'z_min': -0.1, 'z_max': 0.4
        }
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/dobot/state', 10)
        self.error_pub = self.create_publisher(String, '/dobot/error', 10)
        self.executing_pub = self.create_publisher(Bool, '/dobot/executing', 10)
        
        # Services
        self.srv_move_home = self.create_service(
            Trigger, '/dobot/move_home', self.move_home_callback,
            callback_group=self.callback_group
        )
        
        self.srv_move_ready = self.create_service(
            Trigger, '/dobot/move_ready', self.move_ready_callback,
            callback_group=self.callback_group
        )
        
        self.srv_emergency_stop = self.create_service(
            Trigger, '/dobot/emergency_stop', self.emergency_stop_callback,
            callback_group=self.callback_group
        )
        
        self.srv_reset = self.create_service(
            Trigger, '/dobot/reset', self.reset_callback,
            callback_group=self.callback_group
        )
        
        self.srv_execute_pick_place = self.create_service(
            Trigger, '/dobot/execute_pick_place', self.execute_pick_place_callback,
            callback_group=self.callback_group
        )
        
        self.srv_gripper_open = self.create_service(
            Trigger, '/dobot/gripper_open', self.gripper_open_callback,
            callback_group=self.callback_group
        )
        
        self.srv_gripper_close = self.create_service(
            Trigger, '/dobot/gripper_close', self.gripper_close_callback,
            callback_group=self.callback_group
        )
        
        # Timer for state publishing
        self.state_timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info("Dobot Motion Planner initialized successfully for Jazzy")
        
    def joint_state_callback(self, msg):
        """Update current joint states"""
        self.current_joints = list(msg.position)
        
    def publish_state(self):
        """Publish current robot state"""
        msg = String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)
        
        exec_msg = Bool()
        exec_msg.data = (self.current_state == RobotStateEnum.EXECUTING)
        self.executing_pub.publish(exec_msg)
        
    def set_state(self, state: RobotStateEnum):
        """Update robot state"""
        self.current_state = state
        self.get_logger().info(f"State changed to: {state.value}")
        
    def publish_error(self, error_msg: str):
        """Publish error message"""
        msg = String()
        msg.data = error_msg
        self.error_pub.publish(msg)
        self.get_logger().error(error_msg)
        
    def check_workspace_limits(self, pose: Pose) -> bool:
        """Check if pose is within workspace limits"""
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        
        if not (self.workspace_limits['x_min'] <= x <= self.workspace_limits['x_max']):
            return False
        if not (self.workspace_limits['y_min'] <= y <= self.workspace_limits['y_max']):
            return False
        if not (self.workspace_limits['z_min'] <= z <= self.workspace_limits['z_max']):
            return False
            
        return True
        
    def plan_to_joint_state(self, joint_positions: list) -> bool:
        """Plan motion to joint state"""
        if self.emergency_stop_active:
            self.publish_error("Emergency stop active. Reset required.")
            return False
            
        self.set_state(RobotStateEnum.PLANNING)
        
        try:
            # Set joint value target
            self.dobot_arm.set_start_state_to_current_state()
            
            # JAZZY: Updated API for setting goal
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
            joint_dict = dict(zip(joint_names, joint_positions))
            self.dobot_arm.set_goal_state(configuration_name="", 
                                         joint_state_dict=joint_dict)
            
            # Plan
            plan_result = self.dobot_arm.plan()
            
            if plan_result:
                self.get_logger().info("Planning successful")
                return True
            else:
                self.publish_error("Planning failed")
                self.set_state(RobotStateEnum.ERROR)
                return False
                
        except Exception as e:
            self.publish_error(f"Planning exception: {str(e)}")
            self.set_state(RobotStateEnum.ERROR)
            return False
            
    def plan_to_pose(self, pose: Pose) -> bool:
        """Plan motion to Cartesian pose"""
        if self.emergency_stop_active:
            self.publish_error("Emergency stop active. Reset required.")
            return False
            
        if not self.check_workspace_limits(pose):
            self.publish_error("Target pose outside workspace limits")
            return False
            
        self.set_state(RobotStateEnum.PLANNING)
        
        try:
            # Create pose stamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose
            
            # Set pose target
            self.dobot_arm.set_start_state_to_current_state()
            self.dobot_arm.set_goal_state(pose_stamped_msg=pose_stamped, 
                                         pose_link="end_effector")
            
            # Plan
            plan_result = self.dobot_arm.plan()
            
            if plan_result:
                self.get_logger().info("Planning successful")
                return True
            else:
                self.publish_error("Planning failed")
                self.set_state(RobotStateEnum.ERROR)
                return False
                
        except Exception as e:
            self.publish_error(f"Planning exception: {str(e)}")
            self.set_state(RobotStateEnum.ERROR)
            return False
            
    def execute_plan(self) -> bool:
        """Execute the planned trajectory"""
        if self.emergency_stop_active:
            self.publish_error("Emergency stop active. Cannot execute.")
            return False
            
        self.set_state(RobotStateEnum.EXECUTING)
        
        try:
            # Execute
            result = self.dobot_arm.execute()
            
            if result:
                self.get_logger().info("Execution successful")
                self.set_state(RobotStateEnum.IDLE)
                return True
            else:
                self.publish_error("Execution failed")
                self.set_state(RobotStateEnum.ERROR)
                return False
                
        except Exception as e:
            self.publish_error(f"Execution exception: {str(e)}")
            self.set_state(RobotStateEnum.ERROR)
            return False
            
    def move_home_callback(self, request, response):
        """Move to home position"""
        self.get_logger().info("Moving to home position")
        
        if self.plan_to_joint_state(self.predefined_poses['home']):
            if self.execute_plan():
                response.success = True
                response.message = "Moved to home position"
            else:
                response.success = False
                response.message = "Execution failed"
        else:
            response.success = False
            response.message = "Planning failed"
            
        return response
        
    def move_ready_callback(self, request, response):
        """Move to ready position"""
        self.get_logger().info("Moving to ready position")
        
        if self.plan_to_joint_state(self.predefined_poses['ready']):
            if self.execute_plan():
                response.success = True
                response.message = "Moved to ready position"
            else:
                response.success = False
                response.message = "Execution failed"
        else:
            response.success = False
            response.message = "Planning failed"
            
        return response
        
    def execute_pick_place_callback(self, request, response):
        """Execute pick and place sequence"""
        self.get_logger().info("Executing pick and place sequence")
        
        sequence = [
            ('ready', self.predefined_poses['ready']),
            ('pick_approach', self.create_pose_from_xyz(*self.predefined_poses['pick_approach'][:3])),
            ('pick', self.create_pose_from_xyz(*self.predefined_poses['pick'][:3])),
        ]
        
        for name, target in sequence:
            self.get_logger().info(f"Moving to {name}")
            
            if isinstance(target, list):
                success = self.plan_to_joint_state(target)
            else:
                success = self.plan_to_pose(target)
                
            if not success or not self.execute_plan():
                response.success = False
                response.message = f"Failed at {name}"
                return response
                
            # Close gripper at pick position
            if name == 'pick':
                self.gripper_close()
                time.sleep(0.5)
                
        # Move to place
        place_sequence = [
            ('place_approach', self.create_pose_from_xyz(*self.predefined_poses['place_approach'][:3])),
            ('place', self.create_pose_from_xyz(*self.predefined_poses['place'][:3])),
        ]
        
        for name, target in place_sequence:
            self.get_logger().info(f"Moving to {name}")
            
            if not self.plan_to_pose(target) or not self.execute_plan():
                response.success = False
                response.message = f"Failed at {name}"
                return response
                
            # Open gripper at place position
            if name == 'place':
                self.gripper_open()
                time.sleep(0.5)
                
        # Return home
        if self.plan_to_joint_state(self.predefined_poses['home']):
            self.execute_plan()
            
        response.success = True
        response.message = "Pick and place completed"
        return response
        
    def create_pose_from_xyz(self, x, y, z):
        """Create pose from XYZ coordinates"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose
        
    def gripper_open_callback(self, request, response):
        """Open gripper"""
        self.gripper_open()
        response.success = True
        response.message = "Gripper opened"
        return response
        
    def gripper_close_callback(self, request, response):
        """Close gripper"""
        self.gripper_close()
        response.success = True
        response.message = "Gripper closed"
        return response
        
    def gripper_open(self):
        """Open gripper - implement based on your gripper control"""
        self.get_logger().info("Opening gripper")
        # TODO: Implement gripper control for your specific hardware
        
    def gripper_close(self):
        """Close gripper - implement based on your gripper control"""
        self.get_logger().info("Closing gripper")
        # TODO: Implement gripper control for your specific hardware
        
    def emergency_stop_callback(self, request, response):
        """Emergency stop"""
        self.get_logger().warn("EMERGENCY STOP ACTIVATED")
        self.emergency_stop_active = True
        self.set_state(RobotStateEnum.EMERGENCY_STOP)
        
        # TODO: Send stop command to robot hardware
        
        response.success = True
        response.message = "Emergency stop activated"
        return response
        
    def reset_callback(self, request, response):
        """Reset from emergency stop"""
        self.get_logger().info("Resetting from emergency stop")
        self.emergency_stop_active = False
        self.set_state(RobotStateEnum.IDLE)
        
        response.success = True
        response.message = "Robot reset"
        return response


def main(args=None):
    rclpy.init(args=args)
    
    motion_planner = DobotMotionPlanner()
    
    executor = MultiThreadedExecutor()
    executor.add_node(motion_planner)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        motion_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()