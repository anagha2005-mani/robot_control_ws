#!/usr/bin/env python3
"""
Dobot Magician Web Server
Flask-based web interface for controlling the Dobot
Compatible with ROS 2 Jazzy
"""

import threading
import time
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool, Float64MultiArray

from flask import Flask, render_template, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit
from ament_index_python.packages import get_package_share_directory


class DobotWebServer(Node):
    def __init__(self):
        super().__init__('dobot_web_server')

        # -------------------------
        # Flask + SocketIO setup
        # -------------------------
        package_share = get_package_share_directory('dobot_frontend')
        template_dir = os.path.join(package_share, 'templates')  # only templates needed

        self.app = Flask(__name__, template_folder=template_dir)
        CORS(self.app)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")

        # -------------------------
        # Robot state
        # -------------------------
        self.current_state = "unknown"
        self.current_joints = [0.0, 0.0, 0.0, 0.0]
        self.is_executing = False
        self.last_error = ""
        self.connection_status = "disconnected"

        # -------------------------
        # ROS service clients
        # -------------------------
        self._service_clients = {}
        self._setup_service_clients()  # <-- make sure this exists

        # -------------------------
        # ROS subscribers
        # -------------------------
        self.create_subscription(String, '/dobot/state', self._state_callback, 10)
        self.create_subscription(String, '/dobot/error', self._error_callback, 10)
        self.create_subscription(Bool, '/dobot/executing', self._executing_callback, 10)
        self.create_subscription(Float64MultiArray, '/dobot/joint_positions', self._joint_positions_callback, 10)

        # -------------------------
        # Web routes and sockets
        # -------------------------
        self._setup_routes()
        self._setup_socketio()

        # Broadcast state periodically
        self.create_timer(0.1, self._broadcast_state)

        self.get_logger().info("✅ Dobot Web Server initialized")

    # =====================================================
    # ROS SERVICE CLIENTS
    # =====================================================
    def _setup_service_clients(self):
        """
        Create ROS2 service clients for Dobot commands.
        """
        services = [
            'move_home',
            'move_ready',
            'emergency_stop',
            'reset',
            'execute_pick_place',
            'gripper_open',
            'gripper_close'
        ]

        for name in services:
            self._service_clients[name] = self.create_client(
                Trigger,
                f'/dobot/{name}'
            )

    def _wait_for_service(self, name, timeout=2.0):
        client = self._service_clients.get(name)
        if client is None:
            return False
        return client.wait_for_service(timeout_sec=timeout)

    def _call_service_async(self, name):
        client = self._service_clients.get(name)
        if client is None:
            self.get_logger().error(f"Service '{name}' not found")
            return None

        if not self._wait_for_service(name):
            self.get_logger().error(f"Service '{name}' not available")
            return None

        return client.call_async(Trigger.Request())

    # =====================================================
    # ROS CALLBACKS
    # =====================================================
    def _state_callback(self, msg):
        self.current_state = msg.data
        self.connection_status = "connected"

    def _error_callback(self, msg):
        self.last_error = msg.data

    def _executing_callback(self, msg):
        self.is_executing = msg.data

    def _joint_positions_callback(self, msg):
        self.current_joints = list(msg.data)

    # =====================================================
    # FLASK ROUTES
    # =====================================================
    def _setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.app.route('/api/status')
        def status():
            return jsonify({
                'state': self.current_state,
                'joints': self.current_joints,
                'executing': self.is_executing,
                'error': self.last_error,
                'connection': self.connection_status
            })

        # Command endpoints
        @self.app.route('/api/move_home', methods=['POST'])
        def move_home():
            return self._handle_simple_service('move_home')

        @self.app.route('/api/move_ready', methods=['POST'])
        def move_ready():
            return self._handle_simple_service('move_ready')

        @self.app.route('/api/emergency_stop', methods=['POST'])
        def emergency_stop():
            return self._handle_simple_service('emergency_stop')

        @self.app.route('/api/reset', methods=['POST'])
        def reset():
            return self._handle_simple_service('reset')

        @self.app.route('/api/pick_place', methods=['POST'])
        def pick_place():
            return self._handle_simple_service('execute_pick_place')

        @self.app.route('/api/gripper_open', methods=['POST'])
        def gripper_open():
            return self._handle_simple_service('gripper_open')

        @self.app.route('/api/gripper_close', methods=['POST'])
        def gripper_close():
            return self._handle_simple_service('gripper_close')

    def _handle_simple_service(self, name):
        future = self._call_service_async(name)
        if future is None:
            return jsonify({'success': False, 'message': 'Service unavailable'})
        return jsonify({'success': True, 'message': f'{name} command sent'})

    # =====================================================
    # SOCKET.IO
    # =====================================================
    def _setup_socketio(self):
        @self.socketio.on('connect')
        def on_connect():
            self.get_logger().info("🌐 Web client connected")
            emit('connected', {'status': 'connected'})

        @self.socketio.on('disconnect')
        def on_disconnect():
            self.get_logger().info("🌐 Web client disconnected")

        @self.socketio.on('request_state')
        def on_request_state():
            emit('state_update', {
                'state': self.current_state,
                'joints': self.current_joints,
                'executing': self.is_executing,
                'error': self.last_error,
                'connection': self.connection_status
            })

    # =====================================================
    # STATE BROADCAST
    # =====================================================
    def _broadcast_state(self):
        self.socketio.emit('state_update', {
            'state': self.current_state,
            'joints': self.current_joints,
            'executing': self.is_executing,
            'error': self.last_error,
            'connection': self.connection_status,
            'timestamp': time.time()
        })

    # =====================================================
    # FLASK THREAD
    # =====================================================
    def run_flask(self):
        self.socketio.run(
            self.app,
            host='0.0.0.0',
            port=5000,
            debug=False,
            use_reloader=False
        )


# =====================================================
# MAIN
# =====================================================
def main(args=None):
    rclpy.init(args=args)

    node = DobotWebServer()

    flask_thread = threading.Thread(
        target=node.run_flask,
        daemon=True
    )
    flask_thread.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()