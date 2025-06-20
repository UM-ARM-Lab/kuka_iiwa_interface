#!/usr/bin/env python3
"""
Victor Robot State API for Simulator

This module provides a Python API for simulator scripts to communicate with the
victor_sim_hardware interface over ROS topics. It replicates the structure of the
real robot setup and provides function APIs for simulators to:
- Read motion commands from controllers
- Write robot states (MotionStatus) to the hardware interface
- Handle gripper commands and status

Topic Structure:
- Standard victor API topics (for controllers):
  - /victor/{side}_arm/motion_status (published by this API)
  - /victor/{side}_arm/gripper_command (subscribed by this API)
  - /victor/{side}_arm/gripper_status (published by this API)

- Simulator bridge topics (for hardware interface communication):
  - /victor_sim_bridge/{side}/motion_command (subscribed by this API)
  - /victor_sim_bridge/{side}/motion_status (published by this API)
  - /victor_sim_bridge/{side}/gripper_command (published by this API)
  - /victor_sim_bridge/{side}/gripper_status (subscribed by this API)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
from typing import Dict, List, Optional, Callable
import numpy as np

# Victor hardware interface messages
from victor_hardware_interfaces.msg import (
    MotionStatus,
    JointValueQuantity,
    Robotiq3FingerCommand,
    Robotiq3FingerStatus,
    Robotiq3FingerActuatorStatus,
    Robotiq3FingerObjectStatus
)
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion


class VictorSimulatorAPI(Node):
    """
    Main API class for Victor robot simulation.
    
    This class provides a clean interface for Python simulators to communicate
    with the victor_sim_hardware interface, replicating the real robot's behavior.
    """
    
    def __init__(self, node_name: str = "victor_simulator_api", auto_init_rclpy: bool = True):
        # Initialize rclpy if needed and requested
        self._rclpy_initialized_by_us = False
        if auto_init_rclpy and not rclpy.ok():
            rclpy.init()
            self._rclpy_initialized_by_us = True
        
        super().__init__(node_name)
        
        # Use reentrant callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize arm APIs
        self.left_arm = ArmAPI(self, "left")
        self.right_arm = ArmAPI(self, "right")
        
        # Use MultiThreadedExecutor instead of simple spinning
        self._executor = None
        self._executor_thread = None
        self._running = False
        
        self.get_logger().info("Victor Simulator API initialized")
    
    def __del__(self):
        """Destructor to ensure graceful shutdown."""
        try:
            self.stop()
            if self._rclpy_initialized_by_us and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Don't raise exceptions in destructor
            pass
    
    def start(self):
        """Start the API and begin processing ROS callbacks."""
        if self._running:
            return
        
        self._running = True
        
        # Create a dedicated executor for this node
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        
        # Start executor in a separate thread
        self._executor_thread = threading.Thread(target=self._executor_run, daemon=True)
        self._executor_thread.start()
        
        self.get_logger().info("Victor Simulator API started")
    
    def _executor_run(self):
        """Run the executor in a separate thread."""
        try:
            while self._running and rclpy.ok():
                # Spin with timeout to allow for clean shutdown
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            if self._running:  # Only log if we're still supposed to be running
                self.get_logger().error(f"Error in executor thread: {e}")
    
    def stop(self):
        """Stop the API and cleanup resources."""
        if not self._running:
            return
            
        self._running = False
        
        if self._executor:
            self._executor.shutdown()
            
        if self._executor_thread and self._executor_thread.is_alive():
            self._executor_thread.join(timeout=5.0)
            
        self.get_logger().info("Victor Simulator API stopped")
    
    def get_left_arm(self) -> 'ArmAPI':
        """Get the left arm API."""
        return self.left_arm
    
    def get_right_arm(self) -> 'ArmAPI':
        """Get the right arm API."""
        return self.right_arm


class ArmAPI:
    """
    API for a single arm (left or right) of the Victor robot.
    
    This class handles all communication for one arm, including:
    - Motion command reception from controllers
    - Motion status publication to hardware interface
    - Gripper command/status handling
    """
    
    def __init__(self, node: Node, side: str):
        self.node = node
        self.side = side  # "left" or "right"
        
        # Current state
        self._joint_positions = np.zeros(7)
        self._joint_velocities = np.zeros(7)
        self._joint_efforts = np.zeros(7)
        self._external_torques = np.zeros(7)
        self._cartesian_pose = Pose()
        
        # Gripper state
        self._gripper_status = Robotiq3FingerStatus()
        self._init_gripper_status()
        
        # Command callbacks
        self._motion_command_callback: Optional[Callable] = None
        self._gripper_command_callback: Optional[Callable] = None
        
        # Latest received commands
        self._latest_motion_command = None
        self._updated_motion_command = False
        self._latest_gripper_command = None
        self._updated_gripper_command = False
        
        self._setup_publishers()
        self._setup_subscribers()
        
        self.node.get_logger().info(f"Initialized {side} arm API")
    
    def _setup_publishers(self):
        """Setup ROS publishers for this arm."""
        
        # Simulator bridge publishers (to hardware interface)
        self.sim_motion_status_pub = self.node.create_publisher(
            MotionStatus,
            f'/victor_sim_bridge/{self.side}/motion_status',
            10,
            callback_group=self.node.callback_group
        )
        
        self.sim_gripper_status_pub = self.node.create_publisher(
            Robotiq3FingerStatus,
            f'/victor_sim_bridge/{self.side}/gripper_status',
            10,
            callback_group=self.node.callback_group
        )
    
    def _setup_subscribers(self):
        """Setup ROS subscribers for this arm."""
        # Standard victor API subscribers (from controllers)
        self.gripper_command_sub = self.node.create_subscription(
            Robotiq3FingerCommand,
            f'/victor/{self.side}_arm/gripper_command',
            self._gripper_command_ros_callback,
            10,
            callback_group=self.node.callback_group
        )
        
        # Simulator bridge subscribers (from hardware interface)
        self.sim_motion_command_sub = self.node.create_subscription(
            JointValueQuantity,
            f'/victor_sim_bridge/{self.side}/motion_command',
            self._motion_command_sim_callback,
            10,
            callback_group=self.node.callback_group
        )
    
    def _init_gripper_status(self):
        """Initialize gripper status with default values."""
        self._gripper_status.header = Header()
        self._gripper_status.header.frame_id = f"victor_{self.side}_palm"
        
        # Initialize finger status
        for finger_name in ['finger_a_status', 'finger_b_status', 'finger_c_status', 'scissor_status']:
            finger_status = Robotiq3FingerActuatorStatus()
            finger_status.position_request = 0.0
            finger_status.position = 0.0
            finger_status.current = 0.0
            finger_status.header = Header()
            setattr(self._gripper_status, finger_name, finger_status)
        
        # Initialize object status
        for obj_name in ['finger_a_object_status', 'finger_b_object_status', 
                        'finger_c_object_status', 'scissor_object_status']:
            obj_status = Robotiq3FingerObjectStatus()
            obj_status.status = Robotiq3FingerObjectStatus.IN_MOTION
            obj_status.header = Header()
            setattr(self._gripper_status, obj_name, obj_status)
        
        # Initialize gripper system status
        self._gripper_status.initialization_status = Robotiq3FingerStatus.GRIPPER_ACTIVATION
        self._gripper_status.gripper_action_status = Robotiq3FingerStatus.GRIPPER_GOTO
        self._gripper_status.gripper_system_status = Robotiq3FingerStatus.GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE
        self._gripper_status.gripper_motion_status = Robotiq3FingerStatus.GRIPPER_STOPPED_UNKNOWN
        self._gripper_status.gripper_fault_status = Robotiq3FingerStatus.NO_FAULTS
    
    def _motion_command_sim_callback(self, msg: JointValueQuantity):
        """Handle motion command from hardware interface."""
        self._updated_motion_command = True
        self._latest_motion_command = self._extract_joint_positions_from_joint_value_quantity(msg)
    
    def _gripper_command_ros_callback(self, msg: Robotiq3FingerCommand):
        """Handle gripper command from controllers."""
        self._updated_gripper_command = True
        self._latest_gripper_command = self._extract_gripper_positions_from_command(msg)
    
    def _extract_joint_positions_from_joint_value_quantity(self, msg: JointValueQuantity) -> List:
        """Extract joint positions from JointValueQuantity message as list."""
        return [
            msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4,
            msg.joint_5, msg.joint_6, msg.joint_7
        ]
    
    def _extract_gripper_positions_from_command(self, msg: Robotiq3FingerCommand) -> List:
        """Extract gripper finger positions from command as numpy array [a, b, c, scissor]."""
        return [
            msg.finger_a_command.position,
            msg.finger_b_command.position, 
            msg.finger_c_command.position,
            msg.scissor_command.position
        ]

    def get_latest_motion_command(self) -> List|None:
        """Get the latest motion command received from controllers."""
        if self._updated_motion_command and self._latest_motion_command is not None:
            self._updated_motion_command = False
            # Return a copy to avoid external modification
            return self._latest_motion_command.copy()
        return None
    
    def get_latest_gripper_command(self) -> List|None:
        """Get the latest gripper command received from controllers."""
        if self._updated_gripper_command and self._latest_gripper_command is not None:
            self._updated_gripper_command = False
            # Return a copy to avoid external modification
            return self._latest_gripper_command.copy()
        return None
    
    def _create_joint_value_quantity(self, values: List) -> JointValueQuantity:
        """Create a JointValueQuantity message from numpy array."""
        jvq = JointValueQuantity()
        for i in range(7):
            setattr(jvq, f'joint_{i+1}', float(values[i]))
        return jvq
    
    def set_arm_state(self,
        positions: List|None = None,
        velocities: List|None = None,
        efforts: List|None = None,
        external_torques: List|None = None,
        cartesian_pose: List|None = None
    ):
        # If no update, return
        if (
            positions is None and \
            velocities is None and \
            efforts is None and \
            external_torques is None and \
            cartesian_pose is None
        ):
            return
        
        # Set the new values
        for arr, quant in zip(
            [positions, velocities, efforts, external_torques],
            ["_joint_positions", "_joint_velocities", "_joint_efforts", "_external_torques"]
        ):
            if arr is None: continue
            assert len(arr) == 7, "Expected array of shape (7,)"
            setattr(self, quant, arr)
        if cartesian_pose is not None:
            self._cartesian_pose.position = Point(
                x=float(cartesian_pose[0]), 
                y=float(cartesian_pose[1]), 
                z=float(cartesian_pose[2])
            )
            self._cartesian_pose.orientation = Quaternion(
                x=float(cartesian_pose[0]), 
                y=float(cartesian_pose[1]),                                                         
                z=float(cartesian_pose[2]), 
                w=float(cartesian_pose[3])
            )

        # Create motion status message
        msg = MotionStatus()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"victor_{self.side}_arm_world_frame_kuka"
        
        # Set joint values
        msg.measured_joint_position = self._create_joint_value_quantity(self._joint_positions)
        msg.measured_joint_velocity = self._create_joint_value_quantity(self._joint_velocities)
        msg.measured_joint_torque = self._create_joint_value_quantity(self._joint_efforts)
        msg.estimated_external_torque = self._create_joint_value_quantity(self._external_torques)
        # Set cartesian pose
        msg.measured_cartesian_pose = self._cartesian_pose
        
        # Set commanded values (copy from measured for simulation)
        msg.commanded_joint_position = msg.measured_joint_position
        msg.commanded_cartesian_pose = msg.measured_cartesian_pose
        
        # Publish to both topics
        self.sim_motion_status_pub.publish(msg)


    def set_gripper_positions(self,
        finger_a: float,
        finger_b: float,
        finger_c: float,
        scissor: float
    ):
        """
        Set gripper finger positions.
        """
        for val in [finger_a, finger_b, finger_c, scissor]:
            assert isinstance(val, float)
        self._gripper_status.finger_a_status.position = finger_a
        self._gripper_status.finger_b_status.position = finger_b
        self._gripper_status.finger_c_status.position = finger_c
        self._gripper_status.scissor_status.position = scissor

        # Update header timestamp
        self._gripper_status.header.stamp = self.node.get_clock().now().to_msg()
        
        # Publish to both topics
        self.sim_gripper_status_pub.publish(self._gripper_status)


def create_victor_simulator() -> VictorSimulatorAPI:
    """
    Factory function to create and initialize a Victor simulator API.
    
    Returns:
        VictorSimulatorAPI: Initialized simulator API
    """
    # Check if rclpy is already initialized, don't reinitialize
    simulator = VictorSimulatorAPI(auto_init_rclpy=False)
    simulator.start()
    return simulator
    return simulator
