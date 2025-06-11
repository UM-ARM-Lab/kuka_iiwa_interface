#!/usr/bin/env python3
"""
Simple example script showing how to use the Victor Simulator API.

This example demonstrates:
1. Setting up the simulator API
2. Handling motion commands from controllers
3. Publishing robot state updates
4. Managing gripper commands and status

Usage:
    python3 simple_simulator_example.py
"""

import time
import math
import numpy as np
import rclpy
from victor_sim_hardware.robot_state_api import create_victor_simulator


class SimpleJointCommandGenerator:
    """
    A simple joint command generator that demonstrates the Victor Simulator API.
    
    This generator creates sinusoidal joint movements from -1 to 1 radian
    and tracks update frequency.
    """
    
    def __init__(self):
        # Create the victor simulator API
        self.simulator = create_victor_simulator()
        
        # Simulation parameters
        self.dt = 0.001  # 1ms time step
        self.sim_time = 0.0
        
        # Frequency tracking
        self.last_update_time = time.time()
        self.update_count = 0
        self.frequency_samples = []
        
        # Joint command generation parameters
        self.amplitude = 1.0  # -1 to 1 radian
        self.frequency = 0.1  # Hz (slow movement)
        
        print("Simple joint command generator initialized")
    
    def _left_motion_callback(self, msg):
        """Handle motion commands for left arm."""
        pass  # Not needed for command generation
    
    def _right_motion_callback(self, msg):
        """Handle motion commands for right arm."""
        pass  # Not needed for command generation
    
    def _left_gripper_callback(self, msg):
        """Handle gripper commands for left arm."""
        pass  # Not needed for command generation
    
    def _right_gripper_callback(self, msg):
        """Handle gripper commands for right arm."""
        pass  # Not needed for command generation
    
    def generate_joint_commands(self):
        """Generate sinusoidal joint commands from -1 to 1 radian."""
        # Generate sinusoidal joint positions for all 7 joints
        # Each joint has a slightly different phase to create interesting motion
        joint_positions = []
        for i in range(7):
            phase_offset = i * np.pi / 7  # Different phase for each joint
            position = self.amplitude * np.sin(2 * np.pi * self.frequency * self.sim_time + phase_offset)
            joint_positions.append(position)
        
        return joint_positions
    
    def calculate_frequency(self):
        """Calculate and return the current update frequency."""
        current_time = time.time()
        if hasattr(self, 'last_update_time'):
            dt = current_time - self.last_update_time
            if dt > 0:
                freq = 1.0 / dt
                self.frequency_samples.append(freq)
                
                # Keep only last 100 samples for rolling average
                if len(self.frequency_samples) > 100:
                    self.frequency_samples.pop(0)
                
                avg_freq = np.mean(self.frequency_samples)
                return avg_freq
        
        self.last_update_time = current_time
        return 0.0
    
    def run_generation_step(self):
        """Run one command generation step."""
        # Generate joint commands
        joint_positions = self.generate_joint_commands()
        
        # Set joint positions for both arms
        self.simulator.left_arm.set_joint_positions(joint_positions)
        self.simulator.left_arm.set_joint_velocities([0.0] * 7)  # Zero velocities
        self.simulator.left_arm.set_joint_efforts([0.0] * 7)  # Zero efforts
        self.simulator.left_arm.set_external_torques([0.0] * 7)  # No external torques
        
        self.simulator.right_arm.set_joint_positions(joint_positions)
        self.simulator.right_arm.set_joint_velocities([0.0] * 7)
        self.simulator.right_arm.set_joint_efforts([0.0] * 7)
        self.simulator.right_arm.set_external_torques([0.0] * 7)
        
        # Set simple cartesian poses
        self.simulator.left_arm.set_cartesian_pose([0.5, 0.2, 0.3], [0, 0, 0, 1])
        self.simulator.right_arm.set_cartesian_pose([0.5, -0.2, 0.3], [0, 0, 0, 1])
        
        # Publish state to ROS
        self.simulator.left_arm.publish_motion_status()
        self.simulator.right_arm.publish_motion_status()
        self.simulator.left_arm.publish_gripper_status()
        self.simulator.right_arm.publish_gripper_status()
        
        # Update simulation time
        self.sim_time += self.dt
        self.update_count += 1
    
    def run(self):
        """Main command generation loop."""
        print("Starting joint command generation...")
        print("Press Ctrl+C to stop")
        
        try:
            while rclpy.ok():
                start_time = time.time()
                
                # Run generation step
                self.run_generation_step()
                
                # Calculate frequency
                frequency = self.calculate_frequency()
                
                # Sleep to maintain real-time execution
                elapsed = time.time() - start_time
                sleep_time = max(0, self.dt - elapsed)
                time.sleep(sleep_time)
                
                # Print frequency every 1000 updates (~1 second)
                if self.update_count % 1000 == 0:
                    joint_pos = self.generate_joint_commands()
                    print(f"Update frequency: {frequency:.1f} Hz | Joints: [{joint_pos[0]:.3f}, {joint_pos[1]:.3f}, {joint_pos[2]:.3f}, {joint_pos[3]:.3f}, {joint_pos[4]:.3f}, {joint_pos[5]:.3f}, {joint_pos[6]:.3f}]")
        
        except KeyboardInterrupt:
            print("\nShutting down command generator...")
        
        finally:
            self.simulator.stop()
            rclpy.shutdown()


def main():
    """Main entry point."""
    print("Victor Joint Command Generator")
    print("=============================")
    
    # Create and run command generator
    generator = SimpleJointCommandGenerator()
    generator.run()


if __name__ == "__main__":
    main()
