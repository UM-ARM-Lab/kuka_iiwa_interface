#!/usr/bin/env python3
"""
Command follower example script for the Victor Simulator API.

This example demonstrates:
1. Initializing all joints to 1.0 radian
2. Responding to motion commands from controllers
3. Setting joint positions to commanded values
4. Publishing updated robot state

Usage:
    python3 command_follower_example.py
"""

import time

import rclpy
from victor_sim_hardware.robot_state_api import create_victor_simulator


class CommandFollowerExample:
    """
    A command follower that responds to joint commands from controllers.
    
    This example initializes all joints to 1.0 radian and then updates
    joint positions based on received commands.
    """
    
    def __init__(self):
        # Create the victor simulator API
        self.simulator = create_victor_simulator()
        
        # Initialize joint positions to 1.0 radian for all joints
        self.left_joint_positions = [0.5] * 7
        self.right_joint_positions = [0.5] * 7
        
        # Set initial joint positions
        self.simulator.left_arm.set_joint_positions(self.left_joint_positions)
        self.simulator.left_arm.set_joint_velocities([0.0] * 7)
        self.simulator.left_arm.set_joint_efforts([0.0] * 7)
        self.simulator.left_arm.set_external_torques([0.0] * 7)
        
        self.simulator.right_arm.set_joint_positions(self.right_joint_positions)
        self.simulator.right_arm.set_joint_velocities([0.0] * 7)
        self.simulator.right_arm.set_joint_efforts([0.0] * 7)
        self.simulator.right_arm.set_external_torques([0.0] * 7)
        
        # Set up motion command callbacks
        self.simulator.left_arm.set_motion_command_callback(self._left_motion_callback)
        self.simulator.right_arm.set_motion_command_callback(self._right_motion_callback)
        
        # Set up gripper command callbacks
        self.simulator.left_arm.set_gripper_command_callback(self._left_gripper_callback)
        self.simulator.right_arm.set_gripper_command_callback(self._right_gripper_callback)
        
        # Initial cartesian poses
        self.simulator.left_arm.set_cartesian_pose([0.5, 0.2, 0.3], [0, 0, 0, 1])
        self.simulator.right_arm.set_cartesian_pose([0.5, -0.2, 0.3], [0, 0, 0, 1])
        
        print("Command follower initialized - all joints set to 1.0 radian")
        print(f"Left arm joints: {self.left_joint_positions}")
        print(f"Right arm joints: {self.right_joint_positions}")
    
    def _left_motion_callback(self, commanded_joints):
        """Handle motion commands for left arm."""
        # commanded_joints is now a numpy array of 7 joint positions
        assert len(commanded_joints) == 7, "Commanded joint positions must have 7 elements"
        
        # Update internal buffer with commanded values
        self.left_joint_positions = commanded_joints.tolist()
        
    def _right_motion_callback(self, commanded_joints):
        """Handle motion commands for right arm."""
        # commanded_joints is now a numpy array of 7 joint positions
        assert len(commanded_joints) == 7, "Commanded joint positions must have 7 elements"
        
        # Update internal buffer with commanded values
        self.right_joint_positions = commanded_joints.tolist()

    def _left_gripper_callback(self, gripper_positions):
        """Handle gripper commands for left arm."""
        # gripper_positions is now a numpy array [finger_a, finger_b, finger_c, scissor]
        # print(f"Left gripper command received: a={gripper_positions[0]:.3f}, b={gripper_positions[1]:.3f}, c={gripper_positions[2]:.3f}, scissor={gripper_positions[3]:.3f}")
        # Update gripper state based on command if needed
    
    def _right_gripper_callback(self, gripper_positions):
        """Handle gripper commands for right arm."""
        # gripper_positions is now a numpy array [finger_a, finger_b, finger_c, scissor]
        # print(f"Right gripper command received: a={gripper_positions[0]:.3f}, b={gripper_positions[1]:.3f}, c={gripper_positions[2]:.3f}, scissor={gripper_positions[3]:.3f}")
        # Update gripper state based on command if needed
    
    def publish_state(self):
        """Publish current robot state."""
        # Publish motion status for both arms
        self.simulator.left_arm.publish_motion_status()
        self.simulator.right_arm.publish_motion_status()
        
        # Publish gripper status for both arms
        self.simulator.left_arm.publish_gripper_status()
        self.simulator.right_arm.publish_gripper_status()
    
    def run(self):
        """Main command following loop."""
        print("Starting command follower...")
        print("Waiting for joint commands from controllers...")
        print("Press Ctrl+C to stop")
        
        # Publishing rate (100 Hz)
        dt = 0.01
        
        try:
            while rclpy.ok():
                start_time = time.time()
                
                # Always update simulator with current internal state
                self.simulator.left_arm.set_joint_positions(self.left_joint_positions)
                self.simulator.left_arm.set_joint_velocities([0.0] * 7)
                self.simulator.left_arm.set_joint_efforts([0.0] * 7)
                self.simulator.left_arm.set_external_torques([0.0] * 7)
                
                self.simulator.right_arm.set_joint_positions(self.right_joint_positions)
                self.simulator.right_arm.set_joint_velocities([0.0] * 7)
                self.simulator.right_arm.set_joint_efforts([0.0] * 7)
                self.simulator.right_arm.set_external_torques([0.0] * 7)
                
                # Publish current state
                self.publish_state()
                
                # Sleep to maintain publishing rate
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\nShutting down command follower...")
        
        finally:
            self.simulator.stop()
            rclpy.shutdown()


def main():
    """Main entry point."""
    print("Victor Command Follower Example")
    print("===============================")
    
    # Create and run command follower
    follower = CommandFollowerExample()
    follower.run()


if __name__ == "__main__":
    main()
