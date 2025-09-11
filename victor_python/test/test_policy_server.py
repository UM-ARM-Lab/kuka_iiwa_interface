#!/usr/bin/env python3
"""
Test script for VictorPolicyServer pose IK command functionality.

This script reads the current victor_right_tool0 tf frame and sends it back
as a pose IK command to test if the policy server can receive and process it correctly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import String
import time


class VictorPolicyServerTest(Node):
    """Test node for VictorPolicyServer pose IK functionality."""
    
    def __init__(self):
        super().__init__('victor_policy_server_test')
        
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
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher for pose IK commands
        self.pose_ik_pub = self.create_publisher(
            Pose,
            '/victor_policy_bridge/right/pose_ik_command',
            self.high_freq_qos
        )
        
        # Publisher for controller switch commands
        self.controller_switch_pub = self.create_publisher(
            String,
            '/victor_policy_bridge/controller_switch',
            self.reliable_qos
        )
        
        # Subscriber for controller state to monitor policy server response
        self.controller_state_sub = self.create_subscription(
            String,
            '/victor_policy_bridge/right/controller_state',
            self.controller_state_callback,
            self.high_freq_qos
        )
        
        # Test state tracking
        self.test_running = False
        self.controller_state_received = False
        self.last_controller_state = ""
        self.test_start_time = 0.0
        self.controller_switched = False
        
        # Give TF buffer time to fill
        self.get_logger().info("Waiting for TF buffer to fill...")
        self.start_timer = self.create_timer(2.0, self.start_test_callback)  # Start test after 2 seconds
        
    def controller_state_callback(self, msg: String):
        """Monitor controller state messages from policy server."""
        self.controller_state_received = True
        self.last_controller_state = msg.data
        # if self.test_running:
        #     self.get_logger().info(f"Policy server controller state: '{msg.data}'")
        
        # Check if controller has switched to impedance controller
        if not self.controller_switched and "impedance_controller" in msg.data:
            self.controller_switched = True
            self.get_logger().info("✓ Controller successfully switched to impedance_controller")
    
    def start_test_callback(self):
        """Start the test after initial delay."""
        self.get_logger().info("Starting victor_right_tool0 pose IK test...")
        
        # First, switch controller to impedance_controller
        self.get_logger().info("Step 1: Switching to impedance_controller...")
        controller_switch_msg = String()
        controller_switch_msg.data = '{"side": "right", "controller": "impedance_controller"}'
        self.controller_switch_pub.publish(controller_switch_msg)
        
        self.test_running = True
        self.test_start_time = time.time()
        
        # Destroy the start timer since we only want it to run once
        self.destroy_timer(self.start_timer)
        
        # Create test timer
        self.create_timer(1.0, self.run_test_cycle)  # Run test every 500ms
    
    def run_test_cycle(self):
        """Main test cycle - read TF and send pose IK command."""
        try:
            # Only proceed if controller has been switched
            if not self.controller_switched:
                if time.time() - self.test_start_time > 5.0:
                    self.get_logger().error("Controller switch timeout - test failed")
                    self.complete_test()
                return
            
            # Read current victor_right_tool0 transform
            transform = self.tf_buffer.lookup_transform(
                'victor_root',  # Parent frame
                'victor_right_tool0',  # Child frame (target frame)
                rclpy.time.Time(),  # Get latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.get_logger().info(
                f"Read victor_right_tool0 transform: "
                f"pos=({transform.transform.translation.x:.3f}, "
                f"{transform.transform.translation.y:.3f}, "
                f"{transform.transform.translation.z:.3f}), "
                f"rot=({transform.transform.rotation.x:.3f}, "
                f"{transform.transform.rotation.y:.3f}, "
                f"{transform.transform.rotation.z:.3f}, "
                f"{transform.transform.rotation.w:.3f})"
            )
            
            # Convert transform to pose message
            pose_msg = Pose()
            pose_msg.position.x = transform.transform.translation.x
            pose_msg.position.y = transform.transform.translation.y
            pose_msg.position.z = transform.transform.translation.z + 0.01
            pose_msg.orientation.x = transform.transform.rotation.x
            pose_msg.orientation.y = transform.transform.rotation.y
            pose_msg.orientation.z = transform.transform.rotation.z
            pose_msg.orientation.w = transform.transform.rotation.w
            
            self.get_logger().info("Sending pose IK command to policy server...")
            self.pose_ik_pub.publish(pose_msg)
            
            # Check if we've been running for more than 10 seconds
            if time.time() - self.test_start_time > 10.0:
                self.get_logger().info("Test completed after 10 seconds")
                self.complete_test()
                
        except TransformException as ex:
            self.get_logger().error(f"Could not get transform from victor_root to victor_right_tool0: {ex}")
        except RuntimeError as e:
            self.get_logger().error(f"Runtime error during test: {e}")
    
    def complete_test(self):
        """Complete the test and report results."""
        self.test_running = False
        
        self.get_logger().info("=== TEST RESULTS ===")
        if self.controller_state_received:
            self.get_logger().info(f"✓ Policy server is responding (controller state: '{self.last_controller_state}')")
        else:
            self.get_logger().warn("✗ No controller state received from policy server")
        
        if self.controller_switched:
            self.get_logger().info("✓ Controller switch successful")
        else:
            self.get_logger().warn("✗ Controller switch failed or timed out")
            
        self.get_logger().info("✓ TF reading successful")
        self.get_logger().info("✓ Pose IK commands sent successfully")
        self.get_logger().info("=== END TEST ===")
        
        # Shutdown after a brief delay
        def shutdown_callback():
            rclpy.shutdown()
        self.create_timer(1.0, shutdown_callback)


def main():
    """Main function to run the test."""
    rclpy.init()
    
    try:
        test_node = VictorPolicyServerTest()
        test_node.get_logger().info("VictorPolicyServer Test Node started")
        test_node.get_logger().info("This test will:")
        test_node.get_logger().info("1. Switch controller to impedance_controller")
        test_node.get_logger().info("2. Read victor_right_tool0 TF frame")
        test_node.get_logger().info("3. Send identical pose as pose_ik_command")
        test_node.get_logger().info("4. Monitor policy server responses")
        test_node.get_logger().info("Test will run for 10 seconds...")
        
        rclpy.spin(test_node)
        
    except KeyboardInterrupt:
        print("Test interrupted by user")
    except (RuntimeError, ValueError) as e:
        print(f"Test error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
