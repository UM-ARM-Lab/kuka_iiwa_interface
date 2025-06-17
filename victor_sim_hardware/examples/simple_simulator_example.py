#!/usr/bin/env python3
"""
Simple example script showing how to use the Victor Simulator API.

This example demonstrates:
1. Setting up the simulator API
2. Handling motion commands from controllers
3. Publishing robot state updates
4. Managing gripper commands and status

Usage:
    python3 simple_simulator_example.py [--arm] [--finger]
    
    --arm: Enable arm joint motion generation
    --finger: Enable gripper motion generation
    
    If neither flag is specified, the robot will remain stationary.
"""

import time
import math
import numpy as np
import rclpy
import argparse
from victor_sim_hardware.robot_state_api import create_victor_simulator


class SimpleJointCommandGenerator:
    """
    A simple joint command generator that demonstrates the Victor Simulator API.
    
    This generator creates sinusoidal joint movements from -1 to 1 radian
    and gripper movements from 0 (open) to 1 (closed) and tracks update frequency.
    """
    
    def __init__(self, enable_body_motion: bool = False, enable_finger_motion: bool = False):
        # Create the victor simulator API
        self.simulator = create_victor_simulator()
        
        # Motion control flags
        self.enable_body_motion = enable_body_motion
        self.enable_finger_motion = enable_finger_motion
        
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
        
        # Gripper command generation parameters
        self.gripper_frequency = 0.2  # Hz (slower than joints)
        
        # Initial static positions
        self.static_joint_positions = [0.0] * 7
        self.static_gripper_positions = (0.0, 0.0, 0.0, 0.0)  # All open
        
        print("Simple joint command generator initialized")
        print(f"Body motion: {'ENABLED' if self.enable_body_motion else 'DISABLED'}")
        print(f"Finger motion: {'ENABLED' if self.enable_finger_motion else 'DISABLED'}")
        if not self.enable_body_motion and not self.enable_finger_motion:
            print("Robot will remain stationary (no motion flags specified)")
    
    def generate_joint_commands(self):
        """Generate sinusoidal joint commands from -1 to 1 radian."""
        if not self.enable_body_motion:
            return self.static_joint_positions
        
        # Generate sinusoidal joint positions for all 7 joints
        # Each joint has a slightly different phase to create interesting motion
        joint_positions = []
        for i in range(7):
            phase_offset = i * np.pi / 7  # Different phase for each joint
            position = self.amplitude * np.sin(2 * np.pi * self.frequency * self.sim_time + phase_offset)
            joint_positions.append(position)
        
        return joint_positions
    
    def generate_gripper_commands(self):
        """
        Generate sinusoidal gripper commands.
        
        - Fingers: 0-1 range
        - Scissor: -0.15 to 0.15 range (since we're generating, not receiving commands)
        """
        if not self.enable_finger_motion:
            return self.static_gripper_positions
        
        # Generate base sinusoidal motion for fingers (0 to 1 range)
        finger_base_normalized = 0.5 * (1 + np.sin(2 * np.pi * self.gripper_frequency * self.sim_time))
        
        # All three fingers move together for simplicity
        finger_a = finger_base_normalized
        finger_b = finger_base_normalized  
        finger_c = finger_base_normalized
        
        # Generate scissor motion oscillating between -0.15 and 0.15
        scissor_base_normalized = np.sin(2 * np.pi * self.gripper_frequency * self.sim_time * 1.2)
        scissor = 0.15 * scissor_base_normalized  # Oscillates between -0.15 and 0.15
        
        # Check finger combination constraints for generated commands
        ab_sum = finger_a + finger_b
        ac_sum = finger_a + finger_c
        finger_constraint_active = (ab_sum > 1.0) or (ac_sum > 1.0)
        
        if finger_constraint_active and scissor > 0.0:
            # When fingers are constrained, scissor must be <= 0
            scissor = 0.0
        
        return finger_a, finger_b, finger_c, scissor
    
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
        
        # Generate gripper commands
        finger_a, finger_b, finger_c, scissor = self.generate_gripper_commands()
        
        # Set joint positions for both arms
        self.simulator.left_arm.set_joint_positions(joint_positions)
        self.simulator.left_arm.set_joint_velocities([0.0] * 7)  # Zero velocities
        self.simulator.left_arm.set_joint_efforts([0.0] * 7)  # Zero efforts
        self.simulator.left_arm.set_external_torques([0.0] * 7)  # No external torques
        
        self.simulator.right_arm.set_joint_positions(joint_positions)
        self.simulator.right_arm.set_joint_velocities([0.0] * 7)
        self.simulator.right_arm.set_joint_efforts([0.0] * 7)
        self.simulator.right_arm.set_external_torques([0.0] * 7)
        
        # Set gripper positions for both arms
        self.simulator.left_arm.set_gripper_positions(finger_a, finger_b, finger_c, scissor)
        self.simulator.right_arm.set_gripper_positions(finger_a, finger_b, finger_c, scissor)
        
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
                    finger_a, finger_b, finger_c, scissor = self.generate_gripper_commands()
                    
                    # Create status message showing which motions are active
                    motion_status = []
                    if self.enable_body_motion:
                        motion_status.append("BODY")
                    if self.enable_finger_motion:
                        motion_status.append("FINGER")
                    if not motion_status:
                        motion_status.append("STATIC")
                    
                    print(f"Update frequency: {frequency:.1f} Hz | Mode: {'/'.join(motion_status)} | Joints: [{joint_pos[0]:.3f}, {joint_pos[1]:.3f}, {joint_pos[2]:.3f}, {joint_pos[3]:.3f}, {joint_pos[4]:.3f}, {joint_pos[5]:.3f}, {joint_pos[6]:.3f}] | Gripper: [A:{finger_a:.3f}, B:{finger_b:.3f}, C:{finger_c:.3f}, S:{scissor:.3f}]")
        
        except KeyboardInterrupt:
            print("\nShutting down command generator...")
        
        finally:
            self.simulator.stop()
            rclpy.shutdown()


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Victor Robot Simulator Example',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 simple_simulator_example.py --arm              # Only arm motion
  python3 simple_simulator_example.py --finger            # Only gripper motion  
  python3 simple_simulator_example.py --arm --finger     # Both motions
  python3 simple_simulator_example.py                     # No motion (static)
        """
    )
    
    parser.add_argument('--arm', action='store_true',
                       help='Enable arm joint motion generation')
    parser.add_argument('--finger', action='store_true', 
                       help='Enable gripper motion generation')
    
    return parser.parse_args()


def main():
    """Main entry point."""
    # Parse command-line arguments
    args = parse_arguments()
    
    print("Victor Joint Command Generator")
    print("=============================")
    
    # Create and run command generator with specified motion flags
    generator = SimpleJointCommandGenerator(
        enable_body_motion=args.arm,
        enable_finger_motion=args.finger
    )
    generator.run()


if __name__ == "__main__":
    main()
