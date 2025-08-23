#!/usr/bin/env python3
"""
VictorPolicyClient - A ROS 2 client for interfacing with VictorPolicyServer.

This client provides a high-level interface for policy execution, handling
communication with the VictorPolicyServer through the /victor_policy_bridge/
topic namespace. It supports 100Hz control loops and provides convenient
methods for joint control, gripper control, and status monitoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from threading import Lock
import json
import time
from typing import Dict, Any, Optional, Union, List
import numpy as np
import torch

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from victor_hardware_interfaces.msg import (
    MotionStatus,
    Robotiq3FingerCommand,
    Robotiq3FingerStatus,
    JointValueQuantity,
    Robotiq3FingerActuatorCommand
)

from victor_hardware_interfaces_np import *
import ros2_numpy

class VictorArmPolicyClient:
    """Client for controlling a single arm through the policy bridge."""
    
    def __init__(self, node: Node, side: str, device: Union[str, torch.device] = 'cpu'):
        self.node = node
        self.side = side
        
        # Device configuration for torch tensors
        self.device = torch.device(device)
        
        # Command dimensions
        self.arm_cmd_dim = 7
        self.gripper_cmd_dim = 4
        
        # QoS profiles
        self.high_freq_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Status storage - only store data in callbacks
        self.latest_motion_status = None
        self.latest_gripper_status = None
        self.latest_controller_state = None
        self.status_lock = Lock()
        
        # Controller status tracking - simplified, no events
        self.current_controller = None
        
        # Command caching - store last sent commands
        self.last_joint_cmd = None
        self.last_gripper_cmd = None
        self.last_cartesian_cmd = None
        
        # Setup publishers and subscribers (no callback groups)
        self._setup_publishers()
        self._setup_subscribers()
        
        self.node.get_logger().info(f"VictorArmPolicyClient for {side} arm initialized on device {self.device}")
    
    def _setup_publishers(self):
        """Setup command publishers for this arm."""
        
        # Joint commands
        self.joint_cmd_pub = self.node.create_publisher(
            JointValueQuantity,
            f'/victor_policy_bridge/{self.side}/joint_command',
            self.high_freq_qos
        )
        
        # Gripper commands
        self.gripper_cmd_pub = self.node.create_publisher(
            Robotiq3FingerCommand,
            f'/victor_policy_bridge/{self.side}/gripper_command',
            self.high_freq_qos
        )
        
        # Cartesian pose commands
        self.cartesian_cmd_pub = self.node.create_publisher(
            TransformStamped,
            f'/victor_policy_bridge/{self.side}/cartesian_command',
            self.high_freq_qos
        )
    
    def _setup_subscribers(self):
        """Setup status subscribers for this arm."""
        
        # Motion status subscriber - no callback group
        self.motion_status_sub = self.node.create_subscription(
            MotionStatus,
            f'/victor_policy_bridge/{self.side}/motion_status',
            self.motion_status_callback,
            self.high_freq_qos
        )
        
        # Gripper status subscriber - no callback group
        self.gripper_status_sub = self.node.create_subscription(
            Robotiq3FingerStatus,
            f'/victor_policy_bridge/{self.side}/gripper_status',
            self.gripper_status_callback,
            self.high_freq_qos
        )
        
        # Controller state subscriber - no callback group
        self.controller_state_sub = self.node.create_subscription(
            String,
            f'/victor_policy_bridge/{self.side}/controller_state',
            self.controller_state_callback,
            self.high_freq_qos
        )
    
    # -------------------------------------
    # Controller
    # -------------------------------------
    def controller_state_callback(self, msg: String):
        """Handle controller state updates - only store data."""
        with self.status_lock:
            self.latest_controller_state = msg.data
            self.current_controller = msg.data
            self.node.get_logger().debug(f"{self.side} controller state updated to: {self.current_controller}")
    
    def get_current_controller(self) -> Optional[str]:
        """Get the currently active controller for this arm."""
        with self.status_lock:
            return self.current_controller
    
    # -------------------------------------
    # Motion Status
    # -------------------------------------
    def motion_status_callback(self, msg: MotionStatus):
        """Handle motion status updates - only store data."""
        with self.status_lock:
            self.latest_motion_status = msg
    
    def _get_motion_status(self) -> Optional[MotionStatus]:
        """Get latest motion status."""
        with self.status_lock:
            return self.latest_motion_status
    
    # -------------------------------------
    # Joint Position
    # -------------------------------------
    def get_joint_positions(self) -> Optional[np.ndarray]:
        """Get current joint positions as numpy array."""
        status = self._get_motion_status()
        if status is None:
            return None
        return ros2_numpy.numpify(status.measured_joint_position)

    def get_joint_velocities(self) -> Optional[np.ndarray]:
        """Get current joint velocities as numpy array."""
        status = self._get_motion_status()
        if status is None:
            return None
        return ros2_numpy.numpify(status.measured_joint_velocity)
    
    def get_joint_torques(self) -> Optional[np.ndarray]:
        """Get current joint torques as numpy array."""
        status = self._get_motion_status()
        if status is None:
            return None
        return ros2_numpy.numpify(status.measured_joint_torque)
    
    def get_external_torques(self) -> Optional[np.ndarray]:
        """Get estimated external torques as numpy array."""
        status = self._get_motion_status()
        if status is None:
            return None
        
        return ros2_numpy.numpify(status.estimated_external_torque)
    
    def get_joint_cmd(self) -> Optional[np.ndarray]:
        """Get last sent joint command as numpy array.
        
        Returns:
            Last sent joint positions as 7-element numpy array, or None if no command was sent yet
        """
        return self.last_joint_cmd

    def set_joint_cmd(self, joint_positions: np.ndarray):
        """Set joint command for this arm.

        Args:
            joint_positions: 7-element numpy array of joint positions
        """
        # Check current controller mode
        current_controller = self.get_current_controller()
        if not (current_controller and ("position" in current_controller or "impedance" in current_controller)):
            raise ValueError(f"Joint commands only allowed in position_controller or impedance_controller modes, "
                           f"current mode: {current_controller}")
        
        # Validate input is numpy array with correct shape
        if not isinstance(joint_positions, np.ndarray):
            raise ValueError(f"joint_positions must be numpy.ndarray, got {type(joint_positions)}")
        
        if joint_positions.shape != (self.arm_cmd_dim,):
            raise ValueError(f"Expected joint positions shape ({self.arm_cmd_dim},), got {joint_positions.shape}")
        
        # Cache the command
        self.last_joint_cmd = joint_positions.copy()
        
        # Use ros2_numpy conversion
        msg = ros2_numpy.msgify(JointValueQuantity, joint_positions.astype(np.float64))
        self.joint_cmd_pub.publish(msg)

    # -------------------------------------
    # Cartesian Position
    # -------------------------------------
    def get_cartesian_pose(self) -> Optional[np.ndarray]:
        """Get current cartesian pose as numpy array [x, y, z, a, b, c]."""
        status = self._get_motion_status()
        if status is None:
            return None
        
        return ros2_numpy.numpify(status.measured_cartesian_pose)
    
    def get_cartesian_cmd(self) -> Optional[np.ndarray]:
        """Get last sent cartesian command as numpy array.
        
        Returns:
            Last sent cartesian pose as 7-element numpy array [x, y, z, qx, qy, qz, qw], 
            or None if no command was sent yet
        """
        return self.last_cartesian_cmd
    
    def set_cartesian_cmd(self, pose: Union[np.ndarray, TransformStamped]):
        """Send Cartesian pose command for this arm.
        
        Args:
            pose: 7-element numpy array [x, y, z, qx, qy, qz, qw] or geometry_msgs/TransformStamped
        """
        # Check current controller mode
        current_controller = self.get_current_controller()
        if not (current_controller and "cartesian" in current_controller):
            raise ValueError(f"Cartesian commands only allowed in cartesian_controller mode, "
                           f"current mode: {current_controller}")
        
        if isinstance(pose, TransformStamped):
            # Extract pose array from TransformStamped for caching
            cached_pose = np.array([
                pose.transform.translation.x,
                pose.transform.translation.y, 
                pose.transform.translation.z,
                pose.transform.rotation.x,
                pose.transform.rotation.y,
                pose.transform.rotation.z,
                pose.transform.rotation.w
            ])
            self.last_cartesian_cmd = cached_pose
            msg = pose
        elif isinstance(pose, np.ndarray):
            if pose.shape != (7,):
                raise ValueError(f"Expected pose shape (7,), got {pose.shape}")
            
            # Cache the command
            self.last_cartesian_cmd = pose.copy()
            
            # Use ros2_numpy conversion with proper headers
            msg = ros2_numpy.msgify(TransformStamped, pose.astype(np.float64))
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = f'victor_{self.side}_arm_cartesian_cmd'
            msg.child_frame_id = f'victor_{self.side}_arm_sunrise_palm_surface'
        else:
            raise ValueError(f"pose must be numpy.ndarray or TransformStamped, got {type(pose)}")
        
        self.cartesian_cmd_pub.publish(msg)
    
    # -------------------------------------
    # Gripper Status
    # -------------------------------------
    def _create_actuator_cmd(self, position: float, speed: float = 255.0, force: float = 255.0) -> Robotiq3FingerActuatorCommand:
        """Create a gripper actuator command with position, speed, and force."""
        cmd = Robotiq3FingerActuatorCommand()
        cmd.position = max(0.0, min(1.0, float(position)))
        cmd.speed = max(0.0, min(255.0, float(speed)))  # Speed as float, range 0.0-255.0
        cmd.force = max(0.0, min(255.0, float(force)))  # Force as float, range 0.0-255.0
        return cmd
    
    def gripper_status_callback(self, msg: Robotiq3FingerStatus):
        """Handle gripper status updates - only store data."""
        with self.status_lock:
            self.latest_gripper_status = msg
    
    def get_gripper_status(self) -> Optional[Robotiq3FingerStatus]:
        """Get latest gripper status."""
        with self.status_lock:
            return self.latest_gripper_status
    
    def get_gripper_positions(self) -> Optional[np.ndarray]:
        """Get current gripper positions as numpy array [finger_a, finger_b, finger_c, scissor]."""
        status = self.get_gripper_status()
        if status is None:
            return None
        
        # Extract positions from each finger actuator status
        positions = np.array([
            status.finger_a_status.position,
            status.finger_b_status.position, 
            status.finger_c_status.position,
            status.scissor_status.position
        ])
        return positions
    
    def get_gripper_cmd(self) -> Optional[np.ndarray]:
        """Get last sent gripper command as numpy array.
        
        Returns:
            Last sent gripper positions as 4-element numpy array [finger_a, finger_b, finger_c, scissor], 
            or None if no command was sent yet
        """
        return self.last_gripper_cmd
        
    def set_gripper_cmd(self, gripper_positions: Union[List[float], np.ndarray, torch.Tensor]):
        """Send gripper command for this arm.
        
        Args:
            gripper_positions: 4-element numpy array [finger_a, finger_b, finger_c, scissor]
        """
        # Validate input is numpy array with correct shape
        if not isinstance(gripper_positions, np.ndarray):
            raise ValueError(f"gripper_positions must be numpy.ndarray, got {type(gripper_positions)}")
        
        if gripper_positions.shape != (self.gripper_cmd_dim,):
            raise ValueError(f"Expected gripper positions shape ({self.gripper_cmd_dim},), got {gripper_positions.shape}")
        
        # Clamp positions to valid range [0, 1]
        positions_array = np.clip(gripper_positions, 0.0, 1.0)
        
        # Cache the command
        self.last_gripper_cmd = positions_array.copy()
        
        # Create individual actuator commands with default speed and force
        finger_a_array = np.array([positions_array[0], 255.0, 255.0])  # [position, speed, force]
        finger_b_array = np.array([positions_array[1], 255.0, 255.0])
        finger_c_array = np.array([positions_array[2], 255.0, 255.0])
        scissor_array = np.array([positions_array[3], 255.0, 255.0])
        
        # Create the full gripper command array: [finger_a, finger_b, finger_c, scissor] each with [pos, speed, force]
        full_command_array = np.concatenate([finger_a_array, finger_b_array, finger_c_array, scissor_array])
        
        # Use ros2_numpy conversion
        msg = ros2_numpy.msgify(Robotiq3FingerCommand, full_command_array)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        self.gripper_cmd_pub.publish(msg)

class VictorPolicyClient(Node):
    """
    A ROS 2 client for high-level Victor robot control through VictorPolicyServer.
    
    Features:
    - API-level controller switching coordination
    - Modular left/right arm control
    - Dynamic arm enabling/disabling
    - Real-time status monitoring
    - Thread-safe operation
    - Convenient API for policy execution
    """
    
    def __init__(self, node_name: str = 'victor_policy_client', 
                 enable_left: bool = True, enable_right: bool = True,
                 device: Union[str, torch.device] = 'cpu'):
        super().__init__(node_name)
        
        # Device configuration
        self.device = torch.device(device)
        
        # Store arm configuration for controller switching logic
        self.enable_left = enable_left
        self.enable_right = enable_right
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Simple status tracking - no connection monitoring
        self.latest_combined_status = None
        self.last_status_time = 0.0
        
        # Arm clients
        self.left = None
        self.right = None
        
        if enable_left:
            self.left = VictorArmPolicyClient(self, 'left', device=self.device)
        if enable_right:
            self.right = VictorArmPolicyClient(self, 'right', device=self.device)
        
        # Combined status subscriber - no callback group
        self.combined_status_sub = self.create_subscription(
            String,
            '/victor_policy_bridge/combined_status',
            self.combined_status_callback,
            self.reliable_qos
        )
        
        # Controller switching publisher - centralized at client level
        self.controller_switch_pub = self.create_publisher(
            String,
            '/victor_policy_bridge/controller_switch',
            self.reliable_qos
        )
        
        self.get_logger().info("Subscribed to /victor_policy_bridge/combined_status")
        
        enabled_arms = []
        if self.left:
            enabled_arms.append('left')
        if self.right:
            enabled_arms.append('right')
        
        self.get_logger().info(f"VictorPolicyClient '{node_name}' initialized with arms: {enabled_arms} on device {self.device}")
    
    def combined_status_callback(self, msg: String):
        """Handle combined status updates from server - only store data."""
        try:
            self.get_logger().debug(f"Received combined status message: {msg.data[:100]}...")
            # Only store the data, don't do any processing
            self.latest_combined_status = json.loads(msg.data)
            self.last_status_time = time.time()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse combined status JSON: {e}")
            self.get_logger().error(f"Raw message data: {msg.data[:200]}...")  # Log first 200 chars
        except Exception as e:
            self.get_logger().error(f"Unexpected error in combined_status_callback: {e}")

    def check_server_availability(self) -> bool:
        """Check if the server topics are available."""
        try:
            topic_names_and_types = self.get_topic_names_and_types()
            
            # Check if the combined status topic exists
            combined_status_available = any(
                '/victor_policy_bridge/combined_status' in topic_name 
                for topic_name, _ in topic_names_and_types
            )
            
            if combined_status_available:
                self.get_logger().debug("Server topics detected")
                return True
            else:
                self.get_logger().debug("Server topics not found")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error checking server availability: {e}")
            return False
    
    def wait_for_status(self, timeout: float = 10.0) -> bool:
        """Wait for any status message from server (simple UDP-like check)."""
        start_time = time.time()
        initial_time = self.last_status_time
        
        while time.time() - start_time < timeout:
            # Check if we received any status message
            if self.last_status_time > initial_time:
                # self.get_logger().info("Received status from server!")
                return True
            time.sleep(0.1)  # Simple polling
        return False
    
    # Public API Methods - Main node responsibilities only
    
    def get_combined_status(self) -> Optional[Dict[str, Any]]:
        """Get latest combined status."""
        return self.latest_combined_status
    
    def has_recent_status(self, max_age_seconds: float = 2.0) -> bool:
        """Check if we have recent status from server."""
        if self.last_status_time == 0.0:
            return False
        return (time.time() - self.last_status_time) < max_age_seconds
    
    def get_status_age(self) -> float:
        """Get age of last status message in seconds."""
        if self.last_status_time == 0.0:
            return float('inf')
        return time.time() - self.last_status_time
    
    def set_controller(self, side: str, controller_type: str, timeout: float = 10.0) -> bool:
        """Set controller for specified side(s) using centralized switching.
        
        Args:
            side: "left", "right", or "both"
            controller_type: Target controller mode (e.g., "impedance_controller", "position_controller")
            timeout: Maximum time to wait for controller switch (seconds)
            
        Returns:
            bool: True if controller switched successfully, False on timeout
        """
        if side not in ["left", "right", "both"]:
            raise ValueError(f"Invalid side '{side}'. Must be 'left', 'right', or 'both'")
        
        # Validate that requested sides are enabled
        if side == "both":
            if not (self.enable_left and self.enable_right):
                raise ValueError("Cannot set controller for 'both' sides when not both arms are enabled")
        elif side == "left" and not self.enable_left:
            raise ValueError("Cannot set controller for left arm when left arm is not enabled")
        elif side == "right" and not self.enable_right:
            raise ValueError("Cannot set controller for right arm when right arm is not enabled")
        
        # Check if we're already in the desired controller mode
        if self._check_current_controllers(side, controller_type):
            self.get_logger().info(f"Controller already in {controller_type} mode for {side}")
            return True
        
        # Send controller switch command using same JSON format as server
        switch_command = {
            "side": side,
            "controller": controller_type
        }
        msg = String()
        msg.data = json.dumps(switch_command)
        self.controller_switch_pub.publish(msg)
        
        # Wait for controller switch to complete
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._check_current_controllers(side, controller_type):
                self.get_logger().info(f"Controller successfully switched to: {controller_type} for {side}")
                return True
            time.sleep(0.1)
        # Timeout occurred
        self.get_logger().warn(f"Timeout waiting for controller switch to: {controller_type} for {side}")
        return False
    
    def _check_current_controllers(self, side: str, expected_controller: str) -> bool:
        """Check if the current controllers match the expected controller for the given side(s)."""
        if side == "both":
            left_match = self._check_single_controller("left", expected_controller)
            right_match = self._check_single_controller("right", expected_controller)
            print("Matching:", left_match, right_match)
            return left_match and right_match
        else:
            return self._check_single_controller(side, expected_controller)
    
    def _check_single_controller(self, side: str, expected_controller: str) -> bool:
        """Check if a single arm's controller matches the expected controller."""
        arm_client = self.left if side == "left" else self.right
        if arm_client is None:
            return False
        current_controller = arm_client.get_current_controller()
        return current_controller == expected_controller
    
    
def main(args=None):
    """
    Simple main function for testing the client.
    For examples, use the dedicated examples/policy_client_examples.py file.
    """
    rclpy.init(args=args)
    
    client = None
    try:
        # Create a simple test client
        client = VictorPolicyClient('test_client')
        
        # Check if server topics are available first
        if not client.check_server_availability():
            client.get_logger().error("Server not detected. Please start VictorPolicyServer first.")
            return
        
        client.get_logger().info("Server topics found, waiting for status messages...")
        
        # Spin the node while waiting for status
        import threading
        
        # Use MultiThreadedExecutor but client is single-threaded internally
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(client)
        
        # Start spinning in a separate thread
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        if client.wait_for_status(10.0):
            client.get_logger().info("Received status from server!")
            
            # Print server status
            status = client.get_combined_status()
            if status:
                client.get_logger().info(f"Server status: {status}")
        else:
            client.get_logger().warn("No status received from server - but topics exist, continuing anyway")
        
        # Keep alive for a short time
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        if client:
            client.get_logger().info("Interrupted by user")
    except Exception as e:
        print(f"Client error: {e}")
    finally:
        if client is not None:
            client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()