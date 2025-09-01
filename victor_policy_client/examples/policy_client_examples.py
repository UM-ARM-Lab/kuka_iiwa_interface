#!/usr/bin/env python3
"""
Victor Policy Client Examples

This file contains various examples demonstrating how to use the VictorPolicyClient
for different control scenarios including joint control, Cartesian control,
individual arm control, and tensor operations.
"""

import rclpy
import time
import argparse
from typing import Union
import numpy as np
import torch

from victor_python.victor_policy_client import VictorPolicyClient


class PolicyClientExample:
    """Example usage of VictorPolicyClient for policy execution."""
    
    def __init__(self, device: Union[str, torch.device] = 'cpu'):
        # Initialize client with both arms enabled and specified device
        self.client = VictorPolicyClient('policy_example', enable_left=True, enable_right=True, device=device)
        self.get_logger = self.client.get_logger
        self.device = torch.device(device)
    
    def wait_for_server(self, timeout: float = 10.0) -> bool:
        """Wait for server to be available and sending status."""
        self.get_logger().info("Checking for server availability...")
        
        # Check if server topics exist
        if not self.client.check_server_availability():
            self.get_logger().error("Server topics not found - is VictorPolicyServer running?")
            return False
        
        self.get_logger().info("Server topics found, waiting for status messages...")
        
        # Wait for status messages
        if self.client.wait_for_status(timeout):
            self.get_logger().info("Server is responding!")
            
            # Check what status we received
            status = self.client.get_combined_status()
            if status:
                self.get_logger().info(f"Server status: {status}")
            
            return True
        else:
            self.get_logger().warn("No status received from server, but continuing anyway...")
            return True  # Continue even if no status - server might be in dry run mode
    
    def run_joint_control_example(self):
        """Run an example using joint control with the new API."""
        self.get_logger().info("Starting joint control example...")
        
        # Wait for server
        if not self.wait_for_server(10.0):
            self.get_logger().error("Failed to connect to server")
            return
        
        # Switch to impedance control using new centralized API
        if not self.client.set_controller('both', 'impedance_controller', timeout=10.0):
            self.get_logger().error("Failed to switch controllers")
            return
        
        # Policy loop at 100Hz
        for i in range(1000):  # 10 seconds at 100Hz
            t = i * 0.01  # 100Hz timestep
            
            # Get current robot state for both arms
            if self.client.left:
                left_positions = self.client.left.get_joint_positions()
                left_velocities = self.client.left.get_joint_velocities()
                if left_positions is not None:
                    # Small sinusoidal motion around current position
                    amplitude = 0.01  # Reduced amplitude for safety
                    left_target = left_positions + amplitude * torch.sin(torch.tensor([2 * np.pi * 0.5 * t + j * 0.1 for j in range(7)], device=self.device))
                    self.client.left.send_joint_command(left_target)
            
            if self.client.right:
                right_positions = self.client.right.get_joint_positions()
                right_velocities = self.client.right.get_joint_velocities()
                if right_positions is not None:
                    # Different motion pattern for right arm
                    amplitude = 0.05  # Reduced amplitude for safety
                    right_target = right_positions + amplitude * torch.cos(torch.tensor([2 * np.pi * 0.3 * t + j * 0.2 for j in range(7)], device=self.device))
                    self.client.right.send_joint_command(right_target)
            
            # Send gripper commands
            gripper_cmd = [0.5, 0.5, 0.5, 0.5]  # [finger_a, finger_b, finger_c, scissor]
            if self.client.left:
                self.client.left.send_gripper_command(gripper_cmd)
            if self.client.right:
                self.client.right.send_gripper_command(gripper_cmd)
            
            # Print status every second
            if i % 100 == 0:
                self.get_logger().info(f"Policy step {i}, time: {t:.2f}s")
                if self.client.left and left_positions is not None:
                    self.get_logger().info(f"Left arm position: {left_positions.cpu().numpy()}")
                if self.client.right and right_positions is not None:
                    self.get_logger().info(f"Right arm position: {right_positions.cpu().numpy()}")
                
                # Check if we have recent status from server
                if self.client.has_recent_status():
                    self.get_logger().info("Server status is recent")
                else:
                    self.get_logger().warn(f"Server status is old: {self.client.get_status_age():.1f}s ago")
            
            time.sleep(0.01)  # 100Hz control loop
        
        self.get_logger().info("Joint control example completed")
    
    def run_cartesian_control_example(self):
        """Example of Cartesian control with the new API."""
        self.get_logger().info("Starting Cartesian control example...")
        
        if not self.wait_for_server(10.0):
            self.get_logger().error("Failed to connect to server")
            return
        
        # Switch to cartesian control for the right arm only using new API
        if not self.client.set_controller('right', 'cartesian_controller', timeout=10.0):
            self.get_logger().error("Failed to switch right arm to cartesian controller")
            return
        
        # Cartesian policy loop at 50Hz
        for i in range(500):  # 10 seconds at 50Hz
            t = i * 0.02  # 50Hz timestep
            
            # Create circular motion in Cartesian space
            radius = 0.05
            center = [0.5, 0.2, 0.3]  # Center position
            
            # Circular motion in x-y plane
            x = center[0] + radius * np.cos(2 * np.pi * 0.1 * t)
            y = center[1] + radius * np.sin(2 * np.pi * 0.1 * t)
            z = center[2]
            
            # Fixed orientation (quaternion)
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            
            # Create pose command
            pose = torch.tensor([x, y, z, qx, qy, qz, qw], device=self.device)
            
            try:
                if self.client.right:
                    self.client.right.send_cartesian_command(pose)
            except ValueError as e:
                self.get_logger().error(f"Cartesian command failed: {e}")
                break
            
            # Get current robot state for monitoring
            if self.client.right:
                joint_positions = self.client.right.get_joint_positions()
                joint_torques = self.client.right.get_joint_torques()
                external_torques = self.client.right.get_external_torques()
                
                # Print status every 2 seconds
                if i % 100 == 0:
                    self.get_logger().info(f"Cartesian step {i}, time: {t:.2f}s")
                    self.get_logger().info(f"Target pose: [{x:.3f}, {y:.3f}, {z:.3f}]")
                    if joint_positions is not None:
                        self.get_logger().info(f"Joint positions: {joint_positions.cpu().numpy()}")
                    if external_torques is not None:
                        ext_force_norm = torch.norm(external_torques[:3])
                        self.get_logger().info(f"External force magnitude: {ext_force_norm:.3f}")
            
            time.sleep(0.02)  # 50Hz control loop
        
        self.get_logger().info("Cartesian control example completed")
    
    def run_individual_arm_example(self):
        """Example of controlling arms individually with different controllers using new API."""
        self.get_logger().info("Starting individual arm control example...")
        
        if not self.wait_for_server(10.0):
            self.get_logger().error("Failed to connect to server")
            return
        
        # Individual arm controller switching using new centralized API
        if self.client.left:
            if not self.client.set_controller('left', 'impedance_controller', timeout=10.0):
                self.get_logger().error("Failed to switch left arm controller")
                return
        
        if self.client.right:
            if not self.client.set_controller('right', 'position_controller', timeout=10.0):
                self.get_logger().error("Failed to switch right arm controller")
                return
        
        # Control loop
        for i in range(250):  # 5 seconds at 50Hz
            t = i * 0.02
            
            # Left arm: compliant motion
            if self.client.left:
                left_pos = self.client.left.get_joint_positions()
                if left_pos is not None:
                    # Slow sinusoidal motion
                    amplitude = 0.1
                    offset = amplitude * torch.sin(torch.tensor([2 * np.pi * 0.2 * t + j * 0.1 for j in range(7)], device=self.device))
                    target = left_pos + offset
                    self.client.left.send_joint_command(target)
                    self.client.left.send_gripper_command([0.8, 0.8, 0.8, 0.5])
            
            # Right arm: precise positioning
            if self.client.right:
                right_pos = self.client.right.get_joint_positions()
                if right_pos is not None:
                    # Small precise movements
                    amplitude = 0.05
                    offset = amplitude * torch.cos(torch.tensor([2 * np.pi * 0.3 * t + j * 0.1 for j in range(7)], device=self.device))
                    target = right_pos + offset
                    self.client.right.send_joint_command(target)
                    self.client.right.send_gripper_command([0.2, 0.2, 0.2, 0.3])
            
            # Print status every 2 seconds
            if i % 100 == 0:
                self.get_logger().info(f"Individual control step {i}, time: {t:.2f}s")
                if self.client.left:
                    controller = self.client.left.get_current_controller()
                    self.get_logger().info(f"Left arm controller: {controller}")
                if self.client.right:
                    controller = self.client.right.get_current_controller()
                    self.get_logger().info(f"Right arm controller: {controller}")
            
            time.sleep(0.02)
        
        self.get_logger().info("Individual arm control example completed")
    
    def run_tensor_operations_example(self):
        """Example demonstrating tensor operations with robot state using new API."""
        self.get_logger().info("Starting tensor operations example...")
        
        if not self.wait_for_server(10.0):
            self.get_logger().error("Failed to connect to server")
            return
        
        # Switch to impedance control for both arms using new centralized API
        if not self.client.set_controller('both', 'impedance_controller', timeout=10.0):
            self.get_logger().error("Failed to switch controllers")
            return
        
        # Demonstrate various tensor operations
        for i in range(100):  # 2 seconds at 50Hz
            t = i * 0.02
            
            if self.client.right:
                # Get robot state as tensors
                positions = self.client.right.get_joint_positions()
                velocities = self.client.right.get_joint_velocities()
                torques = self.client.right.get_joint_torques()
                
                if positions is not None and velocities is not None:
                    # Compute target using tensor operations
                    # Simple PD control in joint space
                    target_pos = torch.zeros_like(positions)  # Go to zero configuration
                    pos_error = target_pos - positions
                    vel_error = torch.zeros_like(velocities) - velocities
                    
                    # PD gains
                    kp = 0.1 * torch.ones_like(positions)
                    kd = 0.01 * torch.ones_like(velocities)
                    
                    # Compute control command
                    control_cmd = positions + kp * pos_error + kd * vel_error
                    
                    # Send command
                    self.client.right.send_joint_command(control_cmd)
                    
                    # Gripper control with tensor
                    gripper_target = torch.tensor([0.5, 0.5, 0.5, 0.5], device=self.device)
                    self.client.right.send_gripper_command(gripper_target)
                    
                    # Print tensor info every second
                    if i % 50 == 0:
                        self.get_logger().info(f"Tensor example step {i}")
                        self.get_logger().info(f"Position error norm: {torch.norm(pos_error):.3f}")
                        self.get_logger().info(f"Velocity norm: {torch.norm(vel_error):.3f}")
                        if torques is not None:
                            self.get_logger().info(f"Torque norm: {torch.norm(torques):.3f}")
            
            time.sleep(0.02)
        
        self.get_logger().info("Tensor operations example completed")
    
    def run_mixed_controller_example(self):
        """Example demonstrating mixed controller usage with sequential switching."""
        self.get_logger().info("Starting mixed controller example...")
        
        if not self.wait_for_server(10.0):
            self.get_logger().error("Failed to connect to server")
            return
        
        # Phase 1: Both arms in position control
        self.get_logger().info("Phase 1: Both arms in position control")
        if not self.client.set_controller('both', 'position_controller', timeout=10.0):
            self.get_logger().error("Failed to switch to position control")
            return
        
        # Run position control for 2 seconds
        for i in range(100):  # 2 seconds at 50Hz
            t = i * 0.02
            
            # Simple joint movements for both arms
            if self.client.left:
                left_pos = self.client.left.get_joint_positions()
                if left_pos is not None:
                    target = left_pos + 0.01 * torch.sin(torch.tensor([t + j for j in range(7)], device=self.device))
                    self.client.left.send_joint_command(target)
            
            if self.client.right:
                right_pos = self.client.right.get_joint_positions()
                if right_pos is not None:
                    target = right_pos + 0.01 * torch.cos(torch.tensor([t + j for j in range(7)], device=self.device))
                    self.client.right.send_joint_command(target)
            
            time.sleep(0.02)
        
        # Phase 2: Left in impedance, right in cartesian
        self.get_logger().info("Phase 2: Left in impedance, right in cartesian")
        if not self.client.set_controller('left', 'impedance_controller', timeout=10.0):
            self.get_logger().error("Failed to switch left arm to impedance")
            return
        
        if not self.client.set_controller('right', 'cartesian_controller', timeout=10.0):
            self.get_logger().error("Failed to switch right arm to cartesian")
            return
        
        # Run mixed control for 3 seconds
        for i in range(150):  # 3 seconds at 50Hz
            t = i * 0.02
            
            # Left arm: compliant joint control
            if self.client.left:
                left_pos = self.client.left.get_joint_positions()
                if left_pos is not None:
                    target = left_pos + 0.02 * torch.sin(torch.tensor([0.5 * t + j * 0.2 for j in range(7)], device=self.device))
                    self.client.left.send_joint_command(target)
            
            # Right arm: cartesian control
            if self.client.right:
                x = 0.5 + 0.03 * np.sin(2 * np.pi * 0.2 * t)
                y = 0.2 + 0.03 * np.cos(2 * np.pi * 0.2 * t)
                z = 0.3
                pose = torch.tensor([x, y, z, 0.0, 0.0, 0.0, 1.0], device=self.device)
                try:
                    self.client.right.send_cartesian_command(pose)
                except ValueError as e:
                    self.get_logger().error(f"Cartesian command failed: {e}")
                    break
            
            if i % 50 == 0:
                self.get_logger().info(f"Mixed control step {i}")
            
            time.sleep(0.02)
        
        # Phase 3: Back to both in impedance
        self.get_logger().info("Phase 3: Both arms back to impedance control")
        if not self.client.set_controller('both', 'impedance_controller', timeout=10.0):
            self.get_logger().error("Failed to switch back to impedance control")
            return
        
        # Final phase for 1 second
        for i in range(50):  # 1 second at 50Hz
            if self.client.left:
                left_pos = self.client.left.get_joint_positions()
                if left_pos is not None:
                    self.client.left.send_joint_command(left_pos)  # Hold position
            
            if self.client.right:
                right_pos = self.client.right.get_joint_positions()
                if right_pos is not None:
                    self.client.right.send_joint_command(right_pos)  # Hold position
            
            time.sleep(0.02)
        
        self.get_logger().info("Mixed controller example completed")


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Victor Policy Client Examples')
    parser.add_argument(
        '--example', 
        type=str, 
        choices=['joint', 'cartesian', 'individual', 'tensor', 'mixed', 'all'],
        default='all',
        help='Choose which example to run (default: all)'
    )
    parser.add_argument(
        '--device',
        type=str,
        choices=['cpu', 'cuda'],
        default='auto',
        help='Device to use for tensors (default: auto - cuda if available, else cpu)'
    )
    parser.add_argument(
        '--left-only',
        action='store_true',
        help='Enable only left arm'
    )
    parser.add_argument(
        '--right-only', 
        action='store_true',
        help='Enable only right arm'
    )
    
    # Parse known args to allow ROS args to pass through
    parsed_args, remaining_args = parser.parse_known_args()
    
    # Initialize ROS with remaining args
    rclpy.init(args=remaining_args)
    
    example = None
    try:
        # Determine device
        if parsed_args.device == 'auto':
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            device = parsed_args.device
        
        # Determine which arms to enable
        if parsed_args.left_only and parsed_args.right_only:
            print("Error: Cannot specify both --left-only and --right-only")
            return
        elif parsed_args.left_only:
            enable_left, enable_right = True, False
        elif parsed_args.right_only:
            enable_left, enable_right = False, True
        else:
            enable_left, enable_right = True, True
        
        # Initialize example with specified configuration
        example = PolicyClientExample(device=device)
        example.client = VictorPolicyClient(
            'policy_example', 
            enable_left=enable_left, 
            enable_right=enable_right, 
            device=device
        )
        example.get_logger = example.client.get_logger
        
        # Start ROS spinning in a separate thread
        import threading
        from rclpy.executors import MultiThreadedExecutor
        
        executor = MultiThreadedExecutor()
        executor.add_node(example.client)
        
        # Start spinning in a separate thread
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        example.get_logger().info(f"Running examples on device: {device}")
        example.get_logger().info(f"Arms enabled - Left: {enable_left}, Right: {enable_right}")
        
        # Run selected example(s)
        if parsed_args.example == 'joint':
            example.run_joint_control_example()
        elif parsed_args.example == 'cartesian':
            example.run_cartesian_control_example()
        elif parsed_args.example == 'individual':
            example.run_individual_arm_example()
        elif parsed_args.example == 'tensor':
            example.run_tensor_operations_example()
        elif parsed_args.example == 'mixed':
            example.run_mixed_controller_example()
        elif parsed_args.example == 'all':
            # Run all examples with pauses
            example.get_logger().info("Running all examples...")
            
            example.run_joint_control_example()
            time.sleep(2.0)
            
            if enable_right:  # Cartesian example needs right arm
                example.run_cartesian_control_example()
                time.sleep(2.0)
            
            example.run_individual_arm_example()
            time.sleep(2.0)
            
            if enable_right:  # Tensor example uses right arm
                example.run_tensor_operations_example()
                time.sleep(2.0)
            
            if enable_left and enable_right:  # Mixed example needs both arms
                example.run_mixed_controller_example()
        
        example.get_logger().info("All selected examples completed!")
        
    except KeyboardInterrupt:
        if example:
            example.get_logger().info("Interrupted by user")
    except Exception as e:
        print(f"Client error: {e}")
        if example:
            example.get_logger().error(f"Exception: {e}")
    finally:
        if example is not None:
            example.client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
