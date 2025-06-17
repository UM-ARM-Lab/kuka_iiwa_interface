#!/usr/bin/env python3
"""
Command follower example script for the Victor Simulator API.

This example demonstrates:
1. Initializing all joints to 0.5 radian
2. Polling for motion commands from controllers
3. Setting joint positions to commanded values
4. Publishing updated robot state

Usage:
    python3 command_follower_example.py [--arm] [--finger]
    
    --arm: Enable following arm joint commands
    --finger: Enable following gripper commands
    
    If neither flag is specified, the robot will remain at initial positions.
"""

import time
import argparse
from turtle import right

import rclpy
from victor_sim_hardware.robot_state_api import create_victor_simulator


class CommandFollowerExample:
    """
    A command follower that responds to joint commands from controllers.
    
    This example initializes all joints to 0.5 radian and then updates
    joint positions based on received commands via polling.
    """
    
    def __init__(self, enable_arm_following: bool = False, enable_finger_following: bool = False):
        # Create the victor simulator API
        self.api = create_victor_simulator()
        
        # Motion following flags
        self.enable_arm_following = enable_arm_following
        self.enable_finger_following = enable_finger_following
        
        # Initialize joint positions to 0.5 radian for all joints
        self.left_joint_positions = [0.5] * 7
        self.right_joint_positions = [0.5] * 7
        
        # Initialize gripper positions
        self.left_gripper_positions = [0.0, 0.0, 0.0, 0.0]  # [a, b, c, scissor]
        self.right_gripper_positions = [0.0, 0.0, 0.0, 0.0]
        
        # Set initial joint positions
        self.api.left_arm.set_joint_positions(self.left_joint_positions)
        self.api.left_arm.set_joint_velocities([0.0] * 7)
        self.api.left_arm.set_joint_efforts([0.0] * 7)
        self.api.left_arm.set_external_torques([0.0] * 7)
        
        self.api.right_arm.set_joint_positions(self.right_joint_positions)
        self.api.right_arm.set_joint_velocities([0.0] * 7)
        self.api.right_arm.set_joint_efforts([0.0] * 7)
        self.api.right_arm.set_external_torques([0.0] * 7)
        
        # Set initial gripper positions
        self.api.left_arm.set_gripper_positions(*self.left_gripper_positions)
        self.api.right_arm.set_gripper_positions(*self.right_gripper_positions)
        
        # Initial cartesian poses
        self.api.left_arm.set_cartesian_pose([0.5, 0.2, 0.3], [0, 0, 0, 1])
        self.api.right_arm.set_cartesian_pose([0.5, -0.2, 0.3], [0, 0, 0, 1])
        
        print("Command follower initialized - all joints set to 0.5 radian")
        print(f"Arm following: {'ENABLED' if self.enable_arm_following else 'DISABLED'}")
        print(f"Finger following: {'ENABLED' if self.enable_finger_following else 'DISABLED'}")
        print(f"Left arm joints: {self.left_joint_positions}")
        print(f"Right arm joints: {self.right_joint_positions}")
        if not self.enable_arm_following and not self.enable_finger_following:
            print("Robot will remain at initial positions (no following flags specified)")
    
    def poll_commands(self):
        """Poll for latest commands from controllers."""
        # Poll for motion commands if arm following is enabled
        if self.enable_arm_following:
            left_motion_cmd = self.api.left_arm.get_latest_motion_command()
            if left_motion_cmd is not None:
                self.left_joint_positions = left_motion_cmd.tolist()

            right_motion_cmd = self.api.right_arm.get_latest_motion_command()
            if right_motion_cmd is not None:
                self.right_joint_positions = right_motion_cmd.tolist()
                
        # Poll for gripper commands if finger following is enabled
        if self.enable_finger_following:
            left_gripper_cmd = self.api.left_arm.get_latest_gripper_command()
            if left_gripper_cmd is not None:
                # Update left gripper positions
                left_gripper_cmd[:3] *= 1.5
                self.left_gripper_positions = left_gripper_cmd.tolist()
                # print("Got left gripper:", self.left_gripper_positions)
                
            right_gripper_cmd = self.api.right_arm.get_latest_gripper_command()
            if right_gripper_cmd is not None:
                # Update right gripper positions
                right_gripper_cmd[:3] *= 1.5
                self.right_gripper_positions = right_gripper_cmd.tolist()
                # print("Got right gripper:", self.right_gripper_positions)
    
    def update_robot_state(self):
        """Update robot state with current positions."""
        # Update joint positions
        self.api.left_arm.set_joint_positions(self.left_joint_positions)
        self.api.left_arm.set_joint_velocities([0.0] * 7)
        self.api.left_arm.set_joint_efforts([0.0] * 7)
        self.api.left_arm.set_external_torques([0.0] * 7)
        
        self.api.right_arm.set_joint_positions(self.right_joint_positions)
        self.api.right_arm.set_joint_velocities([0.0] * 7)
        self.api.right_arm.set_joint_efforts([0.0] * 7)
        self.api.right_arm.set_external_torques([0.0] * 7)
        
        # Update gripper positions
        self.api.left_arm.set_gripper_positions(*self.left_gripper_positions)
        self.api.right_arm.set_gripper_positions(*self.right_gripper_positions)
    
    def publish_state(self):
        """Publish current robot state."""
        # Publish motion status for both arms
        self.api.left_arm.publish_motion_status()
        self.api.right_arm.publish_motion_status()
        
        # Publish gripper status for both arms
        self.api.left_arm.publish_gripper_status()
        self.api.right_arm.publish_gripper_status()
    
    def run(self):
        """Main command following loop."""
        print("Starting command follower...")
        print("Polling for commands from controllers...")
        print("Press Ctrl+C to stop")
        
        # Publishing rate (100 Hz)
        dt = 0.01
        update_count = 0
        
        try:
            while rclpy.ok():
                start_time = time.time()
                
                # Poll for latest commands
                self.poll_commands()
                
                # Update robot state
                self.update_robot_state()
                
                # Publish current state
                self.publish_state()
                
                update_count += 1
                
                # Print status every 100 updates (~1 second)
                if update_count % 100 == 0:
                    following_status = []
                    if self.enable_arm_following:
                        following_status.append("ARM")
                    if self.enable_finger_following:
                        following_status.append("FINGER")
                    if not following_status:
                        following_status.append("NONE")
                    
                    # print(f"Following: {'/'.join(following_status)} | "
                    #       f"Left joints: [{self.left_joint_positions[0]:.3f}, {self.left_joint_positions[1]:.3f}, {self.left_joint_positions[2]:.3f}, {self.left_joint_positions[3]:.3f}, {self.left_joint_positions[4]:.3f}, {self.left_joint_positions[5]:.3f}, {self.left_joint_positions[6]:.3f}] | "
                    #       f"Left gripper: [A:{self.left_gripper_positions[0]:.3f}, B:{self.left_gripper_positions[1]:.3f}, C:{self.left_gripper_positions[2]:.3f}, S:{self.left_gripper_positions[3]:.3f}]")
                
                # Sleep to maintain publishing rate
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\nShutting down command follower...")
        
        finally:
            self.api.stop()
            rclpy.shutdown()


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Victor Robot Command Follower Example',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 command_follower_example.py --arm               # Only follow arm commands
  python3 command_follower_example.py --finger            # Only follow gripper commands  
  python3 command_follower_example.py --arm --finger      # Follow both commands
  python3 command_follower_example.py                     # No following (static)
        """
    )
    
    parser.add_argument('--arm', action='store_true',
                       help='Enable following arm joint commands')
    parser.add_argument('--finger', action='store_true', 
                       help='Enable following gripper commands')
    
    return parser.parse_args()


def main():
    """Main entry point."""
    # Parse command-line arguments
    args = parse_arguments()
    
    print("Victor Command Follower Example")
    print("===============================")
    
    # Create and run command follower with specified following flags
    follower = CommandFollowerExample(
        enable_arm_following=args.arm,
        enable_finger_following=args.finger
    )
    follower.run()


if __name__ == "__main__":
    main()
