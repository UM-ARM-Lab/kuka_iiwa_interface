#!/usr/bin/env python3
"""
Victor Robot State API for Simulator

This module provides a Python API for simulator scripts to communicate with the
victor_sim_hardware interface over ROS topics. It replicates the structure of the
real robot setup and provides function APIs for simulators to:
- Read motion commands (as JointValueQuantity) from controllers
- Write robot states (MotionStatus) to the hardware interface
- Handle gripper commands (Robotiq3FingerCommand) and status (Robotiq3FingerStatus)

Topic Structure:
- Standard victor API topics (for controllers):
  - /victor/{side}_arm/motion_status (published by this API)
  - /victor/{side}_arm/gripper_command (subscribed by this API)
  - /victor/{side}_arm/gripper_status (published by this API)

- Simulator bridge topics (for hardware interface communication):
  - /victor_sim_bridge/{side}/motion_command (subscribed by this API) - JointValueQuantity
  - /victor_sim_bridge/{side}/motion_status (published by this API) - MotionStatus
  - /victor_sim_bridge/{side}/gripper_command (published by this API) - Robotiq3FingerCommand
  - /victor_sim_bridge/{side}/gripper_status (subscribed by this API) - Robotiq3FingerStatus
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
    Robotiq3FingerActuatorCommand,
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
        
        # Latest received commands - initialize as numpy arrays
        self._latest_motion_command = None
        self._updated_motion_command = False
        self._latest_gripper_command = None
        self._updated_gripper_command = False
        
        self._setup_publishers()
        self._setup_subscribers()
        
        self.node.get_logger().info(f"Initialized {side} arm API")
    
    def _setup_publishers(self):
        """Setup ROS publishers for this arm."""
        # Standard victor API publishers (for controllers)
        self.motion_status_pub = self.node.create_publisher(
            MotionStatus,
            f'/victor/{self.side}_arm/motion_status',
            10,
            callback_group=self.node.callback_group
        )
        
        self.gripper_status_pub = self.node.create_publisher(
            Robotiq3FingerStatus,
            f'/victor/{self.side}_arm/gripper_status',
            10,
            callback_group=self.node.callback_group
        )
        
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
        """Handle joint value command from hardware interface."""
        # Extract joint positions as numpy array directly from JointValueQuantity
        joint_positions = self._extract_joint_positions_from_joint_value_quantity(msg)
        
        # Store the numpy array and set the flag
        self._updated_motion_command = True
        self._latest_motion_command = joint_positions
        
    def _gripper_command_ros_callback(self, msg: Robotiq3FingerCommand):
        """Handle gripper command from controllers with constraint enforcement and 0.8->1.0 mapping."""
        # Extract original finger positions (0-1 range from ROS)
        finger_a = msg.finger_a_command.position
        finger_b = msg.finger_b_command.position
        finger_c = msg.finger_c_command.position
        scissor_input = max(0.0, min(msg.scissor_command.position, 1.0))

        print("Received gripper command:", finger_a, finger_b, finger_c, scissor_input)
        
        # Scale scissor from 0-1 input to -0.15 to 0.15 output
        # 0.0 -> -0.15, 0.5 -> 0.0, 1.0 -> 0.15
        scissor_scaled = -0.15 + (scissor_input * 0.3)  # 0->-0.15, 0.5->0.0, 1->0.15
        
        # Check finger combination constraints using ORIGINAL commands (0.5 threshold)
        ab_sum_orig = finger_a + finger_b
        ac_sum_orig = finger_a + finger_c
        finger_constraint_active = (ab_sum_orig > 0.6) or (ac_sum_orig > 0.6)
        
        if finger_constraint_active and scissor_scaled > 0.0:
            # When fingers are constrained, scissor must be <= 0
            scissor_scaled = 0.0
        
        # Store the processed command
        self._updated_gripper_command = True
        self._latest_gripper_command = np.array([finger_a, finger_b, finger_c, scissor_scaled])
    
    def _extract_joint_positions_from_joint_value_quantity(self, msg: JointValueQuantity) -> np.ndarray:
        """Extract joint positions from JointValueQuantity message as numpy array."""
        return np.array([
            msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4,
            msg.joint_5, msg.joint_6, msg.joint_7
        ])
    
    def _extract_gripper_positions_from_command(self, msg: Robotiq3FingerCommand) -> np.ndarray:
        """Extract gripper finger positions from command as numpy array [a, b, c, scissor]."""
        return np.array([
            msg.finger_a_command.position,
            msg.finger_b_command.position, 
            msg.finger_c_command.position,
            msg.scissor_command.position
        ])
    
    def get_latest_motion_command(self) -> Optional[np.ndarray]:
        """Get the latest motion command received from controllers as numpy array of joint positions."""
        if self._updated_motion_command and self._latest_motion_command is not None:
            self._updated_motion_command = False
            # Verify it's a numpy array before returning
            return self._latest_motion_command.copy()
        return None
    
    def get_latest_gripper_command(self) -> Optional[np.ndarray]:
        """Get the latest gripper command received from controllers."""
        if self._updated_gripper_command and self._latest_gripper_command is not None:
            self._updated_gripper_command = False
            # Return a copy to avoid external modification
            return self._latest_gripper_command.copy()
        return None
    
    def set_joint_positions(self, positions: List[float]):
        """
        Set the current joint positions.
        
        Args:
            positions: List of 7 joint positions in radians
        """
        if len(positions) != 7:
            raise ValueError("Expected 7 joint positions")
        self._joint_positions = np.array(positions)
    
    def set_joint_velocities(self, velocities: List[float]):
        """
        Set the current joint velocities.
        
        Args:
            velocities: List of 7 joint velocities in rad/s
        """
        if len(velocities) != 7:
            raise ValueError("Expected 7 joint velocities")
        self._joint_velocities = np.array(velocities)
    
    def set_joint_efforts(self, efforts: List[float]):
        """
        Set the current joint efforts.
        
        Args:
            efforts: List of 7 joint efforts in Nm
        """
        if len(efforts) != 7:
            raise ValueError("Expected 7 joint efforts")
        self._joint_efforts = np.array(efforts)
    
    def set_external_torques(self, torques: List[float]):
        """
        Set the current external torques.
        
        Args:
            torques: List of 7 external torques in Nm
        """
        if len(torques) != 7:
            raise ValueError("Expected 7 external torques")
        self._external_torques = np.array(torques)
    
    def set_cartesian_pose(self, position: List[float], orientation: List[float]):
        """
        Set the current cartesian pose.
        
        Args:
            position: [x, y, z] in meters
            orientation: [x, y, z, w] quaternion
        """
        self._cartesian_pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        self._cartesian_pose.orientation = Quaternion(x=float(orientation[0]), y=float(orientation[1]), 
                                                     z=float(orientation[2]), w=float(orientation[3]))
    
    def set_gripper_positions(self, finger_a: float, finger_b: float, finger_c: float, scissor: float):
        """
        Set gripper finger positions with reverse mapping.
        
        Args:
            finger_a: Finger A position (0.0 to 1.0) - will be reverse mapped for status
            finger_b: Finger B position (0.0 to 1.0) - will be reverse mapped for status
            finger_c: Finger C position (0.0 to 1.0) - will be reverse mapped for status
            scissor: Scissor position (-0.15 to 0.15)
        """
        self._gripper_status.finger_a_status.position = finger_a
        self._gripper_status.finger_b_status.position = finger_b
        self._gripper_status.finger_c_status.position = finger_c
        self._gripper_status.scissor_status.position = scissor
    
    def _create_joint_value_quantity(self, values: np.ndarray) -> JointValueQuantity:
        """Create a JointValueQuantity message from numpy array."""
        jvq = JointValueQuantity()
        jvq.joint_1 = float(values[0])
        jvq.joint_2 = float(values[1])
        jvq.joint_3 = float(values[2])
        jvq.joint_4 = float(values[3])
        jvq.joint_5 = float(values[4])
        jvq.joint_6 = float(values[5])
        jvq.joint_7 = float(values[6])
        return jvq
    
    def publish_motion_status(self):
        """
        Publish the current motion status to both standard victor API and simulator bridge.
        This should be called regularly (e.g., at 1000 Hz) to update the hardware interface.
        """
        try:
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
            self.motion_status_pub.publish(msg)
            self.sim_motion_status_pub.publish(msg)
        except Exception as e:
            self.node.get_logger().error(f"Error publishing motion status: {e}")
    
    def publish_gripper_status(self):
        """
        Publish the current gripper status to both standard victor API and simulator bridge.
        """
        try:
            # Update header timestamp
            self._gripper_status.header.stamp = self.node.get_clock().now().to_msg()
            
            # Publish to both topics
            self.gripper_status_pub.publish(self._gripper_status)
            self.sim_gripper_status_pub.publish(self._gripper_status)
        except Exception as e:
            self.node.get_logger().error(f"Error publishing gripper status: {e}")


def create_victor_simulator() -> VictorSimulatorAPI:
    """
    Factory function to create and initialize a Victor simulator API.
    
    Returns:
        VictorSimulatorAPI: Initialized simulator API
    """
    # Initialize rclpy if not already initialized
    if not rclpy.ok():
        rclpy.init()
    
    simulator = VictorSimulatorAPI(auto_init_rclpy=False)
    simulator.start()
    return simulator


# Example usage and helper functions
if __name__ == "__main__":
    # Example of how to use the API
    simulator = create_victor_simulator()
    
    try:
        # # Set up motion command callbacks
        # def left_motion_callback(msg):
        #     print(f"Left arm motion command received: {msg.commanded_joint_position}")
        
        # def right_motion_callback(msg):
        #     print(f"Right arm motion command received: {msg.commanded_joint_position}")

        # Simulate robot state updates
        rate_hz = 100  # Update at 100 Hz
        while rclpy.ok():
            # Update robot state (this would come from your physics simulation)
            current_time = time.time()
            
            # Example: simple sinusoidal motion
            positions = [0.1 * np.sin(current_time + i) for i in range(7)]
            
            # Update both arms
            for arm in [simulator.left_arm, simulator.right_arm]:
                arm.set_joint_positions(positions)
                arm.set_joint_velocities([0.0] * 7)  # Zero velocity for this example
                arm.set_joint_efforts([0.0] * 7)     # Zero effort for this example
                arm.set_external_torques([0.0] * 7)  # Zero external torques
                
                # Publish current state
                arm.publish_motion_status()
                arm.publish_gripper_status()
            
            time.sleep(1.0 / rate_hz)
            
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        simulator.stop()
        rclpy.shutdown()
        simulator.stop()
        rclpy.shutdown()