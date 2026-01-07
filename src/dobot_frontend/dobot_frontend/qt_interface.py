#!/usr/bin/env python3
"""
Dobot Magician Qt5 Interface
Desktop GUI application for controlling the Dobot
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool, Float64MultiArray

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QGroupBox,
                             QTextEdit, QGridLayout, QFrame, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QPalette, QColor, QFont
import threading
import time
import math


class ROS2Worker(QObject, Node):
    """ROS2 worker that runs in a separate thread"""
    
    state_updated = pyqtSignal(str)
    joints_updated = pyqtSignal(list)
    executing_updated = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)
    log_message = pyqtSignal(str, str)  # message, level
    
    def __init__(self):
        QObject.__init__(self)
        Node.__init__(self, 'dobot_qt_interface')
        
        # State variables
        self.current_state = "unknown"
        self.current_joints = [0.0, 0.0, 0.0, 0.0]
        self.is_executing = False
        
        # Service clients
        self.clients = {}
        self.setup_service_clients()
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String,
            '/dobot/state',
            self.state_callback,
            10
        )
        
        self.error_sub = self.create_subscription(
            String,
            '/dobot/error',
            self.error_callback,
            10
        )
        
        self.executing_sub = self.create_subscription(
            Bool,
            '/dobot/executing',
            self.executing_callback,
            10
        )
        
        self.joint_positions_sub = self.create_subscription(
            Float64MultiArray,
            '/dobot/joint_positions',
            self.joint_positions_callback,
            10
        )
        
        self.get_logger().info("ROS2 Worker initialized")
        
    def setup_service_clients(self):
        """Create service clients"""
        services = [
            'move_home',
            'move_ready',
            'emergency_stop',
            'reset',
            'execute_pick_place',
            'gripper_open',
            'gripper_close'
        ]
        
        for service_name in services:
            client = self.create_client(Trigger, f'/dobot/{service_name}')
            self.clients[service_name] = client
            
    def state_callback(self, msg):
        """Update current state"""
        self.current_state = msg.data
        self.state_updated.emit(msg.data)
        
    def error_callback(self, msg):
        """Handle error messages"""
        self.error_occurred.emit(msg.data)
        self.log_message.emit(f"ERROR: {msg.data}", "error")
        
    def executing_callback(self, msg):
        """Update executing status"""
        self.is_executing = msg.data
        self.executing_updated.emit(msg.data)
        
    def joint_positions_callback(self, msg):
        """Update joint positions"""
        self.current_joints = list(msg.data)
        self.joints_updated.emit(self.current_joints)
        
    def call_service(self, service_name):
        """Call a service synchronously"""
        client = self.clients.get(service_name)
        if client is None:
            self.log_message.emit(f"Service {service_name} not found", "error")
            return False
            
        if not client.wait_for_service(timeout_sec=2.0):
            self.log_message.emit(f"Service {service_name} not available", "error")
            return False
            
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Wait for result
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.log_message.emit(f"{service_name}: {response.message}", 
                                 "info" if response.success else "error")
            return response.success
        else:
            self.log_message.emit(f"Service {service_name} call failed", "error")
            return False


class DobotControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Dobot Magician Control Panel")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize ROS2 in a separate thread
        self.ros_worker = None
        self.ros_thread = None
        self.executor = None
        self.init_ros2()
        
        # Setup UI
        self.init_ui()
        
        # Setup timers
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status_indicator)
        self.status_timer.start(100)
        
    def init_ros2(self):
        """Initialize ROS2 in a separate thread"""
        def ros_spin():
            rclpy.init()
            self.ros_worker = ROS2Worker()
            
            # Connect signals
            self.ros_worker.state_updated.connect(self.on_state_updated)
            self.ros_worker.joints_updated.connect(self.on_joints_updated)
            self.ros_worker.executing_updated.connect(self.on_executing_updated)
            self.ros_worker.error_occurred.connect(self.on_error_occurred)
            self.ros_worker.log_message.connect(self.add_log_entry)
            
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.ros_worker)
            
            try:
                self.executor.spin()
            except Exception as e:
                print(f"ROS2 spin error: {e}")
            
        self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Wait a bit for ROS2 to initialize
        time.sleep(1.0)
        
    def init_ui(self):
        """Initialize the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Title
        title = QLabel("🤖 Dobot Magician Control Panel")
        title.setAlignment(Qt.AlignCenter)
        title_font = QFont("Arial", 24, QFont.Bold)
        title.setFont(title_font)
        main_layout.addWidget(title)
        
        # Top section - Status and Controls
        top_layout = QHBoxLayout()
        main_layout.addLayout(top_layout)
        
        # Status Panel
        status_group = self.create_status_panel()
        top_layout.addWidget(status_group)
        
        # Control Panel
        control_group = self.create_control_panel()
        top_layout.addWidget(control_group)
        
        # Middle section - Joint Display
        joint_group = self.create_joint_panel()
        main_layout.addWidget(joint_group)
        
        # Bottom section - Gripper and Log
        bottom_layout = QHBoxLayout()
        main_layout.addLayout(bottom_layout)
        
        # Gripper Control
        gripper_group = self.create_gripper_panel()
        bottom_layout.addWidget(gripper_group)
        
        # Activity Log
        log_group = self.create_log_panel()
        bottom_layout.addWidget(log_group)
        
        self.apply_stylesheet()
        
    def create_status_panel(self):
        """Create status display panel"""
        group = QGroupBox("Robot Status")
        layout = QVBoxLayout()
        
        # Connection Status
        conn_layout = QHBoxLayout()
        self.connection_indicator = QLabel("●")
        self.connection_indicator.setFont(QFont("Arial", 20))
        self.connection_label = QLabel("Connecting...")
        conn_layout.addWidget(self.connection_indicator)
        conn_layout.addWidget(self.connection_label)
        conn_layout.addStretch()
        layout.addLayout(conn_layout)
        
        # Robot State
        state_layout = QHBoxLayout()
        self.state_indicator = QLabel("●")
        self.state_indicator.setFont(QFont("Arial", 20))
        self.state_label = QLabel("State: Unknown")
        state_layout.addWidget(self.state_indicator)
        state_layout.addWidget(self.state_label)
        state_layout.addStretch()
        layout.addLayout(state_layout)
        
        # Executing Status
        exec_layout = QHBoxLayout()
        self.exec_indicator = QLabel("●")
        self.exec_indicator.setFont(QFont("Arial", 20))
        self.exec_label = QLabel("Ready")
        exec_layout.addWidget(self.exec_indicator)
        exec_layout.addWidget(self.exec_label)
        exec_layout.addStretch()
        layout.addLayout(exec_layout)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
        
    def create_control_panel(self):
        """Create motion control panel"""
        group = QGroupBox("Motion Controls")
        layout = QGridLayout()
        
        # Home button
        btn_home = QPushButton("🏠 Home")
        btn_home.clicked.connect(self.move_home)
        btn_home.setMinimumHeight(60)
        layout.addWidget(btn_home, 0, 0)
        
        # Ready button
        btn_ready = QPushButton("✅ Ready")
        btn_ready.clicked.connect(self.move_ready)
        btn_ready.setMinimumHeight(60)
        layout.addWidget(btn_ready, 0, 1)
        
        # Pick & Place button
        btn_pick_place = QPushButton("🔄 Pick & Place")
        btn_pick_place.clicked.connect(self.execute_pick_place)
        btn_pick_place.setMinimumHeight(80)
        layout.addWidget(btn_pick_place, 1, 0, 1, 2)
        
        # Emergency Stop button
        btn_estop = QPushButton("🛑 EMERGENCY STOP")
        btn_estop.clicked.connect(self.emergency_stop)
        btn_estop.setMinimumHeight(60)
        btn_estop.setStyleSheet("background-color: #ff4444; color: white; font-weight: bold;")
        layout.addWidget(btn_estop, 2, 0)
        
        # Reset button
        btn_reset = QPushButton("🔄 Reset")
        btn_reset.clicked.connect(self.reset)
        btn_reset.setMinimumHeight(60)
        layout.addWidget(btn_reset, 2, 1)
        
        group.setLayout(layout)
        return group
        
    def create_joint_panel(self):
        """Create joint position display"""
        group = QGroupBox("Joint Positions")
        layout = QGridLayout()
        
        self.joint_labels = []
        
        for i in range(4):
            joint_frame = QFrame()
            joint_frame.setFrameStyle(QFrame.StyledPanel)
            joint_layout = QVBoxLayout()
            
            name_label = QLabel(f"Joint {i+1}")
            name_label.setAlignment(Qt.AlignCenter)
            
            value_label = QLabel("0.00°")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setFont(QFont("Arial", 18, QFont.Bold))
            
            joint_layout.addWidget(name_label)
            joint_layout.addWidget(value_label)
            joint_frame.setLayout(joint_layout)
            
            layout.addWidget(joint_frame, 0, i)
            self.joint_labels.append(value_label)
            
        group.setLayout(layout)
        return group
        
    def create_gripper_panel(self):
        """Create gripper control panel"""
        group = QGroupBox("Gripper Control")
        layout = QHBoxLayout()
        
        btn_open = QPushButton("✋ Open")
        btn_open.clicked.connect(self.gripper_open)
        btn_open.setMinimumHeight(60)
        
        btn_close = QPushButton("✊ Close")
        btn_close.clicked.connect(self.gripper_close)
        btn_close.setMinimumHeight(60)
        
        layout.addWidget(btn_open)
        layout.addWidget(btn_close)
        
        group.setLayout(layout)
        group.setMaximumWidth(400)
        return group
        
    def create_log_panel(self):
        """Create activity log panel"""
        group = QGroupBox("Activity Log")
        layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #1a1a1a;
                color: #00ff00;
                font-family: 'Courier New';
            }
        """)
        
        layout.addWidget(self.log_text)
        group.setLayout(layout)
        return group
        
    def apply_stylesheet(self):
        """Apply custom stylesheet"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #667eea;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #667eea;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #764ba2;
            }
            QPushButton:pressed {
                background-color: #5568d3;
            }
        """)
        
    def update_status_indicator(self):
        """Update status indicators"""
        # Connection indicator (green if connected)
        self.connection_indicator.setStyleSheet("color: #10b981;")
        self.connection_label.setText("Connected")
        
    def on_state_updated(self, state):
        """Handle state updates"""
        self.state_label.setText(f"State: {state.capitalize()}")
        
        # Update indicator color based on state
        color_map = {
            'idle': '#3b82f6',
            'planning': '#f59e0b',
            'executing': '#f59e0b',
            'error': '#ef4444',
            'emergency_stop': '#ef4444'
        }
        color = color_map.get(state, '#808080')
        self.state_indicator.setStyleSheet(f"color: {color};")
        
    def on_joints_updated(self, joints):
        """Handle joint position updates"""
        for i, (joint, label) in enumerate(zip(joints, self.joint_labels)):
            degrees = math.degrees(joint)
            label.setText(f"{degrees:.2f}°")
            
    def on_executing_updated(self, is_executing):
        """Handle executing status updates"""
        if is_executing:
            self.exec_label.setText("Executing...")
            self.exec_indicator.setStyleSheet("color: #f59e0b;")
        else:
            self.exec_label.setText("Ready")
            self.exec_indicator.setStyleSheet("color: #3b82f6;")
            
    def on_error_occurred(self, error):
        """Handle error messages"""
        self.add_log_entry(f"ERROR: {error}", "error")
        
    def add_log_entry(self, message, level):
        """Add entry to activity log"""
        timestamp = time.strftime("%H:%M:%S")
        
        color_map = {
            'info': '#00ff00',
            'warning': '#ffaa00',
            'error': '#ff4444'
        }
        color = color_map.get(level, '#00ff00')
        
        formatted_message = f'<span style="color: {color};">[{timestamp}] {message}</span>'
        self.log_text.append(formatted_message)
        
    def move_home(self):
        """Move to home position"""
        if self.ros_worker:
            threading.Thread(target=lambda: self.ros_worker.call_service('move_home'), daemon=True).start()
            
    def move_ready(self):
        """Move to ready position"""
        if self.ros_worker:
            threading.Thread(target=lambda: self.ros_worker.call_service('move_ready'), daemon=True).start()
            
    def execute_pick_place(self):
        """Execute pick and place"""
        if self.ros_worker:
            threading.Thread(target=lambda: self.ros_worker.call_service('execute_pick_place'), daemon=True).start()
            
    def emergency_stop(self):
        """Emergency stop"""
        if self.ros_worker:
            threading.Thread(target=lambda: self.ros_worker.call_service('emergency_stop'), daemon=True).start()
            self.add_log_entry("EMERGENCY STOP ACTIVATED!", "error")
            
    def reset(self):
        """Reset robot"""
        if self.ros_worker:
            threading.Thread(target=lambda: self.ros_worker.call_service('reset'), daemon=True).start()
            
    def gripper_open(self):
        """Open gripper"""
        if self.ros_worker:
            threading.Thread(target=lambda: self.ros_worker.call_service('gripper_open'), daemon=True).start()
            
    def gripper_close(self):
        """Close gripper"""
        if self.ros_worker:
            threading.Thread(target=lambda: self.ros_worker.call_service('gripper_close'), daemon=True).start()
            
    def closeEvent(self, event):
        """Handle window close"""
        if self.executor:
            self.executor.shutdown()
        if self.ros_worker:
            self.ros_worker.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = DobotControlGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
