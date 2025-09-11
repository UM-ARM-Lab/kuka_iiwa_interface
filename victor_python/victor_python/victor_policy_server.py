#!/usr/bin/env python3
"""
VictorPolicyServer - A ROS 2 node that provides high-level Victor robot control interface.

This server acts as a bridge between policy clients and the Victor robot hardware,
providing controller switching, joint command translation, and status monitoring
at 100Hz. It operates within the /victor_policy_bridge/ topic namespace and can
be configured to republish arbitrary topics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from threading import Lock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import json
import time
import argparse
from queue import Queue
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped
from victor_hardware_interfaces.msg import (
    MotionStatus, 
    Robotiq3FingerCommand, 
    Robotiq3FingerStatus,
    JointValueQuantity
)
from tf2_ros import Buffer, TransformListener
from arm_utilities.tf2wrapper import TF2Wrapper

from victor_python.victor import Victor


class VictorArmPolicyHandler:
    """Handler for a single arm's policy interface."""
    controller_support_dict = {
        "joint": [
            "position_controller",
            "impedance_controller",
        ],
        "cartesian": [
            "cartesian_controller",
        ],
        "joint_trajectory": [
            "joint_impedance_trajectory_controller",
            "joint_position_trajectory_controller",
        ],
    }
    
    def __init__(self, node: Node, side: str, victor_side, victor_instance):
        self.node = node
        self.side = side
        self.victor_side = victor_side
        self.victor_instance = victor_instance  # Reference to main Victor instance

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
        self.exclusive_callback_group = MutuallyExclusiveCallbackGroup()
        
        # State variables
        self._initialized = False
        self.controller_poll_period = 1.0
        self.arm_ctrl_lock = Lock()
        self.arm_busy = False
        
        # Controller state tracking
        self.current_controller = None
        # self.last_controller_poll_time = time.perf_counter()
        
        # Setup subscribers and publishers
        self._setup_subscribers()
        self._setup_publishers()
        
        self.node.get_logger().info(f"VictorArmPolicyHandler for {side} arm initialized")
    
    def _runtime_init(self, init_controller):
        # Set the controller setter
        # We Assume that controllers are NOT changed externally after initialization
        self._set_ctrl_func = getattr(self.victor_instance, f"set_{self.side}_controller")
        # Check and initialize the current controller state
        self.current_controller = init_controller
        self._initialized = True

    def _setup_subscribers(self):
        """Setup command subscribers for this arm."""
        
        # Joint Value Quantity commands
        self.jvq_sub = self.node.create_subscription(
            JointValueQuantity,
            f'/victor_policy_bridge/{self.side}/joint_command',
            self._process_joint_command,
            self.high_freq_qos,
            callback_group=self.exclusive_callback_group
        )

        self.jvq_ik_sub = self.node.create_subscription(
            Pose,
            f'/victor_policy_bridge/{self.side}/pose_ik_command',
            self._process_pose_ik_command,
            self.high_freq_qos,
            callback_group=self.exclusive_callback_group
        )
        
        # Cartesian pose commands
        self.cartesian_cmd_sub = self.node.create_subscription(
            TransformStamped,
            f'/victor_policy_bridge/{self.side}/cartesian_command',
            self._process_cartesian_command,
            self.high_freq_qos,
            callback_group=self.exclusive_callback_group
        )

        # Gripper commands
        self.gripper_cmd_sub = self.node.create_subscription(
            Robotiq3FingerCommand,
            f'/victor_policy_bridge/{self.side}/gripper_command',
            self._process_gripper_command,
            self.high_freq_qos,
            callback_group=self.exclusive_callback_group
        )
    
    def _setup_publishers(self):
        """Setup status publishers for this arm."""
        
    #     # Motion status publisher
    #     self.motion_status_pub = self.node.create_publisher(
    #         MotionStatus,
    #         f'/victor_policy_bridge/{self.side}/motion_status',
    #         self.high_freq_qos
    #     )

        # Tool pose publisher
        self.tool_pose_pub = self.node.create_publisher(
            TransformStamped,
            f'/victor_policy_bridge/{self.side}/tool_pose',
            self.high_freq_qos
        )
        self.tf_wrapper = TF2Wrapper(self.node)

        # Controller state publisher
        self.controller_state_pub = self.node.create_publisher(
            String,
            f'/victor_policy_bridge/{self.side}/controller_state',
            self.high_freq_qos
        )

        self.status_timer = self.node.create_timer(
            0.01,  # 100Hz - fast enough to track any policy speed
            self._publish_tool_pose
        )

    # --------------------------------------
    # Command processing methods
    # --------------------------------------
    def _process_joint_command(self, msg: JointValueQuantity):
        """Process joint value quantity command."""
        if not self._initialized or self.arm_busy:
            return
        # Check if current controller supports joint commands
        if not self._controller_supports("joint"):
            self.node.get_logger().error(
                f"Joint commands not allowed in {self.current_controller} mode for {self.side} arm"
            )
            return
        # Convert JointValueQuantity to list
        joint_positions = [
            msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4,
            msg.joint_5, msg.joint_6, msg.joint_7
        ]
        self.node.get_logger().info(f"Received JP {joint_positions}")
        with self.arm_ctrl_lock:
            self.victor_side.send_joint_cmd(joint_positions)

    def _process_pose_ik_command(self, msg: Pose):
        if not self._initialized or self.arm_busy:
            return
        if not self._controller_supports("joint"):
            self.node.get_logger().error(
                f"Joint IK commands only allowed in joint_controller mode, "
                f"current mode: {self.current_controller} for {self.side} arm"
            )
            return
        self.node.get_logger().info(f"Received Pose IK command type={type(msg)}")
        with self.arm_ctrl_lock:
            self.victor_instance.move_to_pose(self.side+"_arm", msg)
    
    def _process_cartesian_command(self, msg: TransformStamped):
        """Process Cartesian pose command."""
        if not self._initialized or self.arm_busy:
            return
        if not self._controller_supports("cartesian"):
            self.node.get_logger().error(
                f"Cartesian commands only allowed in cartesian_controller mode, "
                f"current mode: {self.current_controller} for {self.side} arm"
            )
            return
        # Send command directly to Victor - the message is already in the correct format
        with self.arm_ctrl_lock:
            self.victor_side.send_cartesian_cmd(msg)
    
    def _process_gripper_command(self, msg: Robotiq3FingerCommand):
        """Process gripper command."""
        if not self._initialized or self.arm_busy:
            return
        # Extract gripper positions (only finger_a and scissor are used)
        finger_a_position = msg.finger_a_command.position
        scissor_position = msg.scissor_command.position

        # Use Victor's set_gripper_position method
        self.victor_side.set_gripper_position(finger_a_position, scissor_position)
    
    # --------------------------------------
    # Controller state management methods
    # --------------------------------------
    def _controller_supports(self, cmd_type: str) -> bool:
        """Check if current controller supports the given command type."""
        if self.current_controller is None:
            return False
        if cmd_type not in self.controller_support_dict.keys():
            self.node.get_logger().error(
                f"Unsupported command type '{cmd_type}' for {self.side} arm"
            )
            return False
        for ctrl in self.controller_support_dict[cmd_type]:
            if ctrl in self.current_controller:
                return True
        return False

    # --------------------------------------
    # State Publishing methods
    # --------------------------------------
    def publish_controller_state(self):
        """Publish current controller state."""
        controller_msg = String()
        controller_msg.data = self.current_controller if self.current_controller is not None else ""
        self.controller_state_pub.publish(controller_msg)

    def _publish_tool_pose(self):
        # timeout = rclpy.duration.Duration(seconds=1.0)
        transform = self.tf_wrapper.get_transform("victor_root", f"victor_{self.side}_tool0")
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.node.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "victor_root"
        transform_stamped.child_frame_id = f"victor_{self.side}_tool0"
        transform_stamped.transform = transform
        self.tool_pose_pub.publish(transform_stamped)

    # def _publish_motion_status(self):
    #     """Publish current motion status for this arm."""
    #     motion_status = self.victor_side.get_motion_status()
    #     if motion_status is not None:
    #         self.motion_status_pub.publish(motion_status)
    #         self.latest_motion_status = motion_status
        
    # def _publish_gripper_status(self):
    #     gripper_status = self.victor_side.get_gripper_status()
    #     if gripper_status is not None:
    #         self.gripper_status_pub.publish(gripper_status)
    #         self.latest_gripper_status = gripper_status
    
    # def publish_status(self):
    #     """Publish current status for this arm - called from main timer."""
    #     # Try publishing
    #     try:
    #         self._publish_controller_state()
    #         self._publish_motion_status()
    #         self._publish_gripper_status()
    #     except Exception as e:
    #         self.node.get_logger().error(f"Critical error in get_motion_status for {self.side}: {e}")
    #         import traceback
    #         self.node.get_logger().error(f"Full traceback: {traceback.format_exc()}")

    #     # Add debug counter to verify timer is running
    #     if hasattr(self, 'status_last_time'):
    #         elapsed = time.perf_counter() - self.status_last_time
    #         # print(f"{self.side} publish time: {elapsed:.4f} seconds")
    #     self.status_last_time = time.perf_counter()

    
class VictorPolicyServer(Node):
    """
    A ROS 2 server that provides high-level Victor robot control interface for policy execution.
    
    Features:
    - Centralized controller switching at server level
    - Modular left/right arm handlers with independent command processing
    - Dynamic arm enabling/disabling
    - Status monitoring and republishing
    - Configurable topic republishing for any message type
    - Single-threaded execution with proper ordering
    - High-frequency status tracking (100Hz) for any policy speed
    """
    
    def __init__(self):
        super().__init__('victor_policy_server')
        
        # QoS profiles for high-frequency communication
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Declare parameters
        self.declare_parameter('republish_config_file', '')
        self.declare_parameter('left_arm_enabled', True)
        self.declare_parameter('right_arm_enabled', True)

        # Don't initialize Victor or arm handlers yet - wait for parameter override
        self.left_handler = None
        self.right_handler = None
        self.status_timer = None
        self._initialized = False

        # Initialize Victor without executor
        self.victor = Victor(self)
        
        # Status publishing timer - no callback group, single threaded
        self.status_timer = self.create_timer(
            0.01,  # 100Hz - fast enough to track any policy speed
            self.status_loop_callback
        )
        self.status_loop_count = 0
        
        # Combined status publisher
        self.combined_status_pub = self.create_publisher(
            String,
            '/victor_policy_bridge/combined_status',
            self.reliable_qos
        )
        # Controller switching subscriber - centralized at server level
        self.controller_switch_sub = self.create_subscription(
            String,
            f'/victor_policy_bridge/controller_switch',
            self._process_controller_switch,
            self.reliable_qos,
        )
        self.controller_switch_queue = Queue()
        self.controller_switch_timeout = 1.0

        left_enabled = self.get_parameter('left_arm_enabled').get_parameter_value().bool_value
        right_enabled = self.get_parameter('right_arm_enabled').get_parameter_value().bool_value
        
        if left_enabled:
            # No callback group - single threaded
            self.left_handler = VictorArmPolicyHandler(
                self, 'left', self.victor.left,
                victor_instance=self.victor
            )
            self.get_logger().info(f"Left arm handler enabled")
        
        if right_enabled:
            # No callback group - single threaded  
            self.right_handler = VictorArmPolicyHandler(
                self, 'right', self.victor.right, 
                victor_instance=self.victor
            )
            self.get_logger().info(f"Right arm handler enabled")
    
    def _runtime_init(self):
        """Initialize the server after parameters are finalized."""
        if self._initialized:
            return
        
        # Configure controller info
        controller_info = self.victor.get_and_update_active_controller_names()
        
        if self.left_handler and len(controller_info["left"]) > 0:
            self.left_handler._runtime_init(controller_info["left"][0])
        if self.right_handler and len(controller_info["right"]) > 0:
            self.right_handler._runtime_init(controller_info["right"][0])
        
        self._initialized = True
        self.get_logger().info("VictorPolicyServer initialized")

    def status_loop_callback(self):
        """Main processing loop - handles commands and publishes status at 100Hz."""
        if not self._initialized:
            self._runtime_init()

        # Publish combined status as JSON
        combined_status = {
            'timestamp': time.perf_counter(),
            'left_enabled': self.left_handler is not None,
            'right_enabled': self.right_handler is not None,
        }
        status_msg = String()
        status_msg.data = json.dumps(combined_status)
        self.combined_status_pub.publish(status_msg)

        # Publish individual controller states
        if self.left_handler:
            self.left_handler.publish_controller_state()
        if self.right_handler:
            self.right_handler.publish_controller_state()

        # Deal with controller switches
        self._complete_controller_switch()
        # self.status_last_time = time.perf_counter()

    def _process_controller_switch(self, msg: String):
        """Process controller switch commands using centralized switching."""
        if not self._initialized:
            return
        
        # try:
        # Parse JSON command in same format as client sends
        switch_command = json.loads(msg.data.strip())
        
        if "side" not in switch_command or "controller" not in switch_command:
            self.get_logger().error(f"Invalid controller switch command format: {switch_command}")
            return
        
        side = switch_command["side"]
        controller_type = switch_command["controller"]
        
        # Validate side parameter
        if side not in ["left", "right", "both"]:
            self.get_logger().error(f"Invalid side '{side}' in controller switch command")
            return
        
        sides = ['left', 'right'] if side == 'both' else [side]

        # Abandon switch if arm is busy
        if ('left' in sides and self.left_handler and self.left_handler.arm_busy) or \
            ('right' in sides and self.right_handler and self.right_handler.arm_busy):
            self.get_logger().warn(f"Controller switch abandoned - arm busy for {side}")
            return

        # Set arm busy state to prevent new commands
        if 'left' in sides and self.left_handler:
            self.left_handler.arm_busy = True
        if 'right' in sides and self.right_handler:
            self.right_handler.arm_busy = True
        
        # Get async request and put into the queue
        try:
            future = self.victor.set_controller_async(controller_type, side)
            self.controller_switch_queue.put((time.perf_counter(), side, controller_type, future))
            self.get_logger().info(f"Processing controller switch: {side} -> {controller_type}")
        except Exception as e:
            self.get_logger().error(f"Error requesting controller switch: {e}")

    
    def _complete_controller_switch(self):
        while self.controller_switch_queue.qsize() > 0:
            first = self.controller_switch_queue.queue[0]

            future = first[3]
            if future.done():
                # Remove the completed switch from the queue
                self.controller_switch_queue.get()
                if not self.victor.set_controller_async_callback(future).ok:
                    self.get_logger().error(
                        f"Controller failed to switch to {first[2]} for {first[1]}"
                    )
                    if (first[1] == 'left' or first[1] == 'both') and self.left_handler:
                        self.left_handler.arm_busy = False
                    if (first[1] == 'right' or first[1] == 'both') and self.right_handler:
                        self.right_handler.arm_busy = False
                    continue
                self.get_logger().info(
                    f"Controller switched to {first[2]} for {first[1]}"
                )
                if (first[1] == 'left' or first[1] == 'both') and self.left_handler:
                    self.left_handler.arm_busy = False
                    self.left_handler.current_controller = first[2]
                if (first[1] == 'right' or first[1] == 'both') and self.right_handler:
                    self.right_handler.arm_busy = False
                    self.right_handler.current_controller = first[2]
                continue

            # If timeout, throw away
            if time.perf_counter() - first[0] > self.controller_switch_timeout:
                self.controller_switch_queue.get()
                self.get_logger().warn(f"Controller switch timeout for {first[1]} -> {first[2]}")
                if (first[1] == 'left' or first[1] == 'both') and self.left_handler:
                    self.left_handler.arm_busy = False
                if (first[1] == 'right' or first[1] == 'both') and self.right_handler:
                    self.right_handler.arm_busy = False
                continue
            return

def create_victor_policy_server_node(**kwargs):
    """
    Factory function to create a VictorPolicyServer node for integration into launch files.
    
    Args:
        **kwargs: Keyword arguments that will be passed as ROS parameters
                 Supported: left_arm_enabled, right_arm_enabled, republish_config_file
    
    Returns:
        VictorPolicyServer: Initialized server node ready to be added to an executor
    """
    # Create server node
    server = VictorPolicyServer()
    
    # Set parameters from kwargs
    if kwargs:
        override_params = []
        for param_name, param_value in kwargs.items():
            if param_name == 'left_arm_enabled':
                override_params.append(
                    Parameter('left_arm_enabled', Parameter.Type.BOOL, bool(param_value))
                )
            elif param_name == 'right_arm_enabled':
                override_params.append(
                    Parameter('right_arm_enabled', Parameter.Type.BOOL, bool(param_value))
                )
            elif param_name == 'republish_config_file':
                override_params.append(
                    Parameter('republish_config_file', Parameter.Type.STRING, str(param_value))
                )
        
        if override_params:
            server.set_parameters(override_params)
    
    # Log final configuration
    left_enabled = server.get_parameter('left_arm_enabled').get_parameter_value().bool_value
    right_enabled = server.get_parameter('right_arm_enabled').get_parameter_value().bool_value
    
    server.get_logger().info("VictorPolicyServer created with configuration:")
    server.get_logger().info(f"  Left arm: {left_enabled}")
    server.get_logger().info(f"  Right arm: {right_enabled}")
    server.get_logger().info("  Status tracking: 100Hz (supports any policy speed)")
    server.get_logger().info("Ready for external executor")
    
    return server


def main(*args ,**kwargs):
    """
    Main function that creates a VictorPolicyServer node for launch file integration.
    Returns the server node for external executor management.
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Victor Policy Server')
    parser.add_argument(
        '--no-left',
        action='store_true',
        help='Disable left arm'
    )
    parser.add_argument(
        '--no-right',
        action='store_true',
        help='Disable right arm'
    )
    
    # Parse known args to allow ROS args to pass through
    parsed_args, remaining_args = parser.parse_known_args()
    
    # Initialize ROS with remaining args
    rclpy.init(args=remaining_args)
    
    try:
        # Prepare parameters
        kwargs = {}
        if parsed_args.no_left:
            kwargs['left_arm_enabled'] = False
        if parsed_args.no_right:
            kwargs['right_arm_enabled'] = False
        
        # Create server using factory function
        server = create_victor_policy_server_node(**kwargs)
        
        server.get_logger().info("VictorPolicyServer ready for launch file executor")
        
        return server
        
    except Exception as e:
        print(f"Server creation error: {e}")
        rclpy.shutdown()
        raise


if __name__ == '__main__':
    server = main()
    if server:
        try:
            # Create executor and add server
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(server)
            
            server.get_logger().info("VictorPolicyServer running with standalone executor")
            
            # Use the executor for standalone execution
            executor.spin()
        except KeyboardInterrupt:
            print("Interrupted by user")
        except SystemExit:
            pass
        except Exception as e:
            print(f"Executor error: {e}")
        finally:
            try:
                executor.shutdown()
            except Exception as e:
                print(f"Executor shutdown error: {e}")
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"RCL shutdown error: {e}")
        print("Done!")