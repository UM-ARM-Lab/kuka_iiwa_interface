#!/usr/bin/env python3
"""
This script receives teleop commands from the Unity VR system, and sends them to Victor.
Press and hold the grip button on the VR controller to start recording an episode.
Release the grip button to stop recording an episode.
Press the menu button to stop the script.
Use the trigger to open and close the gripper, it's mapped to the open fraction of the gripper.
The motion will be relative in gripper frame to the pose of the gripper when you started recording.
see the vr_ros2_bridge repo for setup instructions.

Usage:
- Set --arms=[left|right|both] to control one or both arms.
- Set --motion=[relative|absolute] to control the motion mode.
- Set --target_controller=[impedance_controller|position_controller] to control the target controller.
"""
from copy import deepcopy
from time import perf_counter
import argparse
import time
import threading
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import transforms3d
from queue import Queue
from threading import Lock
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from ament_index_python import get_package_share_path
# from arm_utilities.filters import BatchOnlineFilter
from arm_utilities.numpy_conversions import transform_to_mat, mat_to_transform
from arm_utilities.transformation_helper import np_tf_inv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from moveit.core.robot_model import RobotModel, JointModelGroup
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from victor_hardware_interfaces.msg import MotionStatus, Robotiq3FingerStatus
from victor_python.victor import Victor, Side
from victor_python.victor_utils import jvq_to_list
from vr_ros2_bridge_msgs.msg import ControllersInfo, ControllerInfo, TrackerInfo
from std_msgs.msg import Bool

from victor_python.victor_vr_teleop_profiles import VictorTeleopProfile, teleop_profile_dict

VR_FRAME_NAME = "vr"

from scipy.signal import firwin
class BatchOnlineFilter:
    def __init__(self, numtaps: int, cutoff: float):
        """
        A simple online FIR low-pass filter.

        Args:
          numtaps: number of filter taps (length of FIR kernel).
          cutoff:  normalized cutoff frequency (0..1, where 1 is Nyquist).
        """
        self.numtaps = numtaps
        # design FIR kernel
        # firwin returns an array of length `numtaps`
        self.coeffs = firwin(numtaps, cutoff, window='hamming')
        # buffer will be lazily initialized to shape (numtaps, *sample_shape)
        self.buffer = None

    def update(self, sample):
        """
        Push a new sample through the filter.

        Args:
          sample: scalar or numpy array of arbitrary shape.
        Returns:
          filtered output, same shape as `sample`.
        """
        x = np.asarray(sample)

        # on first call, set up a zeroed circular buffer of the right shape
        if self.buffer is None:
            # buffer shape: (numtaps, ...) where ... matches sample.shape
            self.buffer = np.stack([x] * self.numtaps, axis=0)
            
        # roll the buffer down one step and insert the newest sample
        self.buffer = np.roll(self.buffer, shift=1, axis=0)
        self.buffer[0] = x

        # compute weighted sum along axis=0:
        #   out = sum_{i=0..numtaps-1} coeffs[i] * buffer[i, ...]
        # Using tensordot handles arbitrary trailing dims.
        out = np.tensordot(self.coeffs, self.buffer, axes=(0, 0))

        return out


def controller_info_to_tf(node: Node, controller_info: ControllerInfo):
    pose_msg: Pose = controller_info.controller_pose
    tf = TransformStamped()
    tf.header.frame_id = VR_FRAME_NAME
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.child_frame_id = controller_info.controller_name
    tf.transform.translation.x = pose_msg.position.x
    tf.transform.translation.y = pose_msg.position.y
    tf.transform.translation.z = pose_msg.position.z
    tf.transform.rotation = pose_msg.orientation
    return tf

def tracker_info_to_tf(node: Node, tracker_info: TrackerInfo):
    pose_msg: Pose = tracker_info.tracker_pose
    tf = TransformStamped()
    tf.header.frame_id = VR_FRAME_NAME
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.child_frame_id = tracker_info.tracker_name
    tf.transform.translation.x = pose_msg.position.x
    tf.transform.translation.y = pose_msg.position.y
    tf.transform.translation.z = pose_msg.position.z
    tf.transform.rotation = pose_msg.orientation
    return tf

def rotation_matrix_gripper_joint(x):
    """
    Returns a 4x4 homogeneous transformation matrix representing a rotation 
    around the local z-axis by angle x (in radians).
    """
    cos_x = np.cos(x)
    sin_x = np.sin(x)
    return np.array([
        [cos_x, -sin_x, 0, 0],
        [sin_x,  cos_x, 0, 0],
        [0,      0,     1, 0],
        [0,      0,     0, 1]
    ])

def _Rx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0,  0],
                        [0, c, -s],
                        [0, s,  c]])

def _Ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c, 0, s],
                        [ 0, 1, 0],
                        [-s, 0, c]])

def _Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                        [s,  c, 0],
                        [0,  0, 1]])

def rotation_matrix_held_tool_joint(center_offset: np.ndarray, rate: np.ndarray):
    """
    Parameters
    ----------
    droll, dpitch, dyaw : float
        Small-angle increments (rad) that already include Δt.

    Returns
    -------
    T : (4,4) ndarray
        Homogeneous transform that realises the requested rotation
        about `self.wrench_center_in_tool`.
    """
    # 1. Compose rotation for this Δt  (ZYX intrinsic convention).
    droll, dpitch, dyaw = rate.tolist()
    R = (_Rz(dyaw) @ _Ry(dpitch) @ _Rx(droll))

    # 2. Express rotation about an arbitrary point p:
    p = center_offset.reshape(3, 1)
    t = (np.eye(3) - R) @ p       # == p - R p

    # 3. Pack into a 4×4 homogeneous matrix.
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t.ravel()
    return T

class ProcessFn:
    def __init__(self,
        name,
        fn,
        when_gripped=False
    ):
        """
        A process function that can be registered to the teleop class.
        It will be called with the controller_info as argument.
        
        Args:
            name: Name of the process function, used for debugging.
            fn: The function to call.
            always: If True, the function will be called every time process_input is called.
        """
        self.name = name
        self.fn = fn
        self.when_gripped = when_gripped
    
    def __call__(self, *args, **kwargs):
        return self.fn(*args, **kwargs)

class BaseVRTracker:
    def __init__(
        self, node: Node, 
        tf_broadcaster: TransformBroadcaster,
        side_name: str, 
        ctrl_profile: VictorTeleopProfile,
        publish_pose=False,
    ):
        self.node = node
        self.side_name = side_name
        self.tf_broadcaster = tf_broadcaster
        self.publish_pose = publish_pose

        # Initialize tracker
        self._init_vr_tracker(ctrl_profile)

    def _init_vr_tracker(self, ctrl_profile):
        """Initialize fundamental VR parameters"""
        # Controller Orientation setup
        self.controller_in_vr0 = np.eye(4)
        self.tool_in_base0 = np.eye(4)
        usb_rot_vec = ctrl_profile.usability_rotation[self.side_name]
        self.usability_rotation = transforms3d.euler.euler2mat(
            *usb_rot_vec
        )

        # Sensitivity setup
        self.position_sensitivity = ctrl_profile.position_sensitivity  # e.g., 1.0
        self.orientation_sensitivity = ctrl_profile.orientation_sensitivity  # e.g., 1.0

        # Filtering motion
        self.use_filter = ctrl_profile.use_filter
        self.filter_pub = self.node.create_publisher(JointState, 'filtered_joint_states', 10)
        self.filter = BatchOnlineFilter(numtaps=20, cutoff=0.1)

    def get_controller_in_vr(self, controller_info):
        controller_in_vr_msg = controller_info_to_tf(self.node, controller_info)
        controller_in_vr = transform_to_mat(controller_in_vr_msg.transform)

        usability_transform = np.eye(4)
        usability_transform[:3, :3] = self.usability_rotation
        controller_in_vr_usable = controller_in_vr @ usability_transform

        return controller_in_vr_usable

    def on_start_recording(self, controller_info: ControllerInfo):
        controller_in_vr0 = self.get_controller_in_vr(controller_info)
        self.controller_in_vr0 = controller_in_vr0

        controller_in_vr0_msg = TransformStamped()
        controller_in_vr0_msg.transform = mat_to_transform(controller_in_vr0)
        controller_in_vr0_msg.header.frame_id = VR_FRAME_NAME
        controller_in_vr0_msg.child_frame_id = f"{self.side_name}_in_vr0"
        self.tf_broadcaster.sendTransform(controller_in_vr0_msg)

    def on_stop_recording(self):
        pass

class ViewportTracker(BaseVRTracker):
    """Viewport Tracker for the Victor robot"""
    def __init__(self, node: Node, tf_broadcaster: TransformBroadcaster, ctrl_profile: VictorTeleopProfile):
        """
        Args:
            node: ROS node
            tf_broadcaster: Publishing TF for visualization
            ctrl_profile: Control profile for the VR tracker
        """
        super().__init__(node, tf_broadcaster, "viewport", ctrl_profile)
        
        self._initialize_pubsub()
        
        # Override sensitivity settings for viewport tracking
        self.position_sensitivity = ctrl_profile.viewport_position_sensitivity
        self.orientation_sensitivity = ctrl_profile.viewport_orientation_sensitivity
        
        # Initialize filter for viewport position
        self.viewport_filter = BatchOnlineFilter(numtaps=40, cutoff=0.1)

    def _initialize_pubsub(self):
        self.cam_pose_sub = self.node.create_subscription(
            Pose,
            "/victor_sim_bridge/viewport_pose_status",
            self.on_camera_pose,
            10
        )
        self.cam_pose_pub = self.node.create_publisher(
            Pose,
            "/victor_sim_bridge/viewport_pose_command",
            10
        )
        
        # Track initial viewport pose (captured once from ROS on first message)
        self.viewport_initial_pose = None
        self.viewport_initial_captured = False
        
        # Track initial tracker pose (captured once on first tracker message)
        self.tracker_initial_pose = None
        self.tracker_initial_captured = False
        
        # Queue for asynchronous processing
        self.tracker_queue = Queue()
        self.processing_thread = None
        self.shutdown_event = False
        
    def on_camera_pose(self, pose_msg: Pose):
        """Handle incoming viewport pose messages - capture initial pose once"""
        if not self.viewport_initial_captured:
            self.viewport_initial_pose = deepcopy(pose_msg)
            print(f"Initial viewport pose - xyz: ({pose_msg.position.x:.3f}, {pose_msg.position.y:.3f}, {pose_msg.position.z:.3f}), "
                  f"quaternion wxyz: ({pose_msg.orientation.w:.3f}, {pose_msg.orientation.x:.3f}, {pose_msg.orientation.y:.3f}, {pose_msg.orientation.z:.3f})")
            self.viewport_initial_captured = True
            self.node.get_logger().info("Captured initial viewport pose for tracking reference")

    def publish_viewport_command(self, target_pose: Pose):
        """Publish a viewport command pose"""
        if self.cam_pose_pub is not None:
            self.cam_pose_pub.publish(target_pose)
            
        # Also publish to TF for visualization
        viewport_tf = TransformStamped()
        viewport_tf.header.stamp = self.node.get_clock().now().to_msg()
        viewport_tf.header.frame_id = "victor_root"
        viewport_tf.child_frame_id = "viewport_target"
        viewport_tf.transform.translation.x = target_pose.position.x
        viewport_tf.transform.translation.y = target_pose.position.y
        viewport_tf.transform.translation.z = target_pose.position.z
        viewport_tf.transform.rotation = target_pose.orientation
        self.tf_broadcaster.sendTransform(viewport_tf)

    def on_stop_recording(self):
        """Override to handle viewport-specific recording stop"""
        pass

    def get_tracker_in_vr(self, tracker_info: TrackerInfo):
        """Convert tracker info to VR coordinate transform"""
        tracker_pose = tracker_info.tracker_pose
        tracker_in_vr = np.eye(4)
        
        # Position
        tracker_in_vr[:3, 3] = [
            tracker_pose.position.x,
            tracker_pose.position.y,  
            tracker_pose.position.z
        ]
        
        # Orientation
        quat = [
            tracker_pose.orientation.x,
            tracker_pose.orientation.y,
            tracker_pose.orientation.z,
            tracker_pose.orientation.w
        ]
        tracker_in_vr[:3, :3] = R.from_quat(quat).as_matrix()
        
        # Apply usability rotation if needed
        usability_transform = np.eye(4)
        usability_transform[:3, :3] = self.usability_rotation
        tracker_in_vr_usable = tracker_in_vr @ usability_transform
        
        return tracker_in_vr_usable
        
    def process_input(self, tracker_info: TrackerInfo):
        """Process head tracker input and publish viewport commands - continuous tracking"""
        # Capture initial tracker pose on first call
        st = perf_counter()
        if not self.tracker_initial_captured:
            self.tracker_initial_pose = self.get_tracker_in_vr(tracker_info)
            self.tracker_initial_captured = True
            
            # Print initial tracker pose
            tracker_pose = tracker_info.tracker_pose
            print(f"Initial ViewportTracker pose - xyz: ({tracker_pose.position.x:.3f}, {tracker_pose.position.y:.3f}, {tracker_pose.position.z:.3f}), "
                  f"quaternion xyzw: ({tracker_pose.orientation.x:.3f}, {tracker_pose.orientation.y:.3f}, {tracker_pose.orientation.z:.3f}, {tracker_pose.orientation.w:.3f})")
            
            self.node.get_logger().info("Captured initial tracker pose for tracking reference")
        
        # Skip processing if we don't have the initial viewport pose yet
        if not self.viewport_initial_captured or self.viewport_initial_pose is None:
            return
            
        # Get current tracker pose in VR
        current_tracker_pose = self.get_tracker_in_vr(tracker_info)
        
        # Compute relative transform from initial tracker pose
        delta_tracker = np_tf_inv(self.tracker_initial_pose) @ current_tracker_pose
        
        # Apply sensitivity scaling
        delta_tracker[:3, 3] *= self.position_sensitivity
        
        # For orientation, scale the rotation angles
        relative_rotation = R.from_matrix(delta_tracker[:3, :3])
        relative_euler = relative_rotation.as_euler('xyz')
        scaled_euler = relative_euler * self.orientation_sensitivity
        delta_tracker[:3, :3] = R.from_euler('xyz', scaled_euler).as_matrix()
        
        # Convert initial viewport pose to transform matrix
        initial_viewport_transform = np.eye(4)
        initial_viewport_transform[:3, 3] = [
            self.viewport_initial_pose.position.x,
            self.viewport_initial_pose.position.y,
            self.viewport_initial_pose.position.z
        ]
        initial_quat = [
            self.viewport_initial_pose.orientation.x,
            self.viewport_initial_pose.orientation.y,
            self.viewport_initial_pose.orientation.z,
            self.viewport_initial_pose.orientation.w
        ]
        initial_viewport_transform[:3, :3] = R.from_quat(initial_quat).as_matrix()
        
        # Apply relative transform to initial viewport pose
        # Use inverse of delta to make viewport move in same direction as head
        ### TESTING
        target_viewport_transform = initial_viewport_transform @ delta_tracker
        
        # Apply filter to position
        if self.use_filter:
            target_position = target_viewport_transform[:3, 3]
            filtered_position = self.viewport_filter.update(target_position)
            target_viewport_transform[:3, 3] = filtered_position
        
        # Convert back to Pose message
        target_pose = Pose()
        target_pose.position.x = target_viewport_transform[0, 3]
        target_pose.position.y = target_viewport_transform[1, 3]
        target_pose.position.z = target_viewport_transform[2, 3]
        
        # Convert rotation matrix to quaternion
        target_quat = R.from_matrix(target_viewport_transform[:3, :3]).as_quat()
        target_pose.orientation.x = target_quat[0]
        target_pose.orientation.y = target_quat[1]
        target_pose.orientation.z = target_quat[2]
        target_pose.orientation.w = target_quat[3]
        
        # Publish viewport command
        self.publish_viewport_command(target_pose)

class SideTeleop(BaseVRTracker):
    """Teleop one side of Victor"""
    def __init__(self, 
            node: Node, 
            side: Side, 
            victor: Victor,
            moveitpy: MoveItPy, 
            tf_broadcaster: TransformBroadcaster,
            ctrl_profile: VictorTeleopProfile
        ):
        """

        Args:
            node: ROS node
            side: Side object, from Victor
            moveitpy: MoveItPy object, there must only be one of there per python script/process!
            tf_broadcaster: Publishing TF for visualization
            controller_usability_rotation: Offset to the VR controller, to make the orientation more unintuitive.

        """
        side_name =  side.arm_name.split('_')[0]  # e.g., 'left' or 'right'
        super().__init__(node, tf_broadcaster, side_name, ctrl_profile)
        self.side = side
        self.victor = victor
        self.moveitpy = moveitpy
        self.robot_model: RobotModel = self.moveitpy.get_robot_model()
        self.base_frame = self.robot_model.model_frame
        self.jmg: JointModelGroup = self.robot_model.get_joint_model_group(self.side.arm_name)
        self.tool_frame = self.jmg.eef_name

        # Processor functions
        self.get_joint_fn = ProcessFn(
            "default_get_joint_positions",
            self.get_joint_positions,
        )
        self.process_fns = {
            # "queue_pose_cmd": ProcessFn("queue_pose_cmd", self.queue_pose_cmd, when_gripped=True),
            "send_pose_cmd": ProcessFn("send_pose_cmd", self.send_pose_cmd, when_gripped=True),
        }

        # Parse control profile
        self._init_gripper(ctrl_profile)
        self._init_robot_controller(ctrl_profile)
        self._init_processing_queue(ctrl_profile)
        self._init_trackpad(ctrl_profile)
        self._init_trackpad_rot(ctrl_profile)
        self._init_correction_maneuver_fn(ctrl_profile)

    # ---------------------------
    # Core functionalities initializers
    # ---------------------------
    def _init_gripper(self, ctrl_profile):
        # Gripper setup
        self.gripper_in_motion = None
        # Keypoints for gripper control
        self.gripper_keypoints = ctrl_profile.gripper_keypoints
        self._max_gripper_idx = len(self.gripper_keypoints)-1
        self.gripper_state_idx = ctrl_profile.init_gripper_state
        # For now, only identical value for finger a,b,c are supported
        self.side.set_gripper_position(self.gripper_keypoints[self.gripper_state_idx][0],
                                       scissor_position=self.gripper_keypoints[self.gripper_state_idx][-1])
        self.toggle_gripper_dt = 0.25  # seconds
        self.last_toggled_gripper = perf_counter()
        self.process_fns['trackpad_updown_gripper'] = ProcessFn(
            "trackpad_updown_gripper",
            self.toggle_gripper_up_down,
            when_gripped=False
        )

    def _init_robot_controller(self, ctrl_profile):
        """Initialize controllers for robot arms"""
        # Then set the controller to the target controller
        activ_ctrl = list(set(self.victor.left.get_active_controller_names()))
        if f"{self.side_name}_{ctrl_profile.target_controller}" not in activ_ctrl:
            ctrl_setter = getattr(self.victor, f'set_{self.side_name}_controller')
            ctrl_setter(ctrl_profile.target_controller)
        # Get controller
        active_controller_names = self.side.get_active_controller_names()
        if len(active_controller_names) != 1:
            raise ValueError(f"Expected exactly one active controller, got {active_controller_names}")
        active_controller_name = active_controller_names[0]
        self.joint_cmd_pub = self.side.get_joint_cmd_pub(active_controller_name)

    def _init_processing_queue(self, ctrl_profile):
        """
        Initialize processing queue for the controller.
        This is used to process the input from the controller.
        """
        self.controller_info_queue = Queue()
        self.commanded_joints_queue = Queue()
        
        # Threading setup for asynchronous processing
        self.max_ctrl_delay = getattr(ctrl_profile, 'max_ctrl_delay', 0.1)  # seconds
        self.worker_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix=f"{self.side_name}_ik_worker")
        self.command_thread = None
        self.shutdown_event = threading.Event()
        
        # Synchronization for ordered command processing
        self.last_queued_time = 0.0
        self.queue_lock = threading.Lock()

    # ---------------------------
    # Additioal functionalities initializers
    # ---------------------------
    def _init_trackpad(self, ctrl_profile):
        """
        Initialize trackpad settings.
        """
        # if |x| or |y| < deadzone we ignore it for region tests
        self.trackpad_click_deadzone = 0.4

    def _init_trackpad_rot(self, ctrl_profile):
        self.trackpad_for_wrist_rot = getattr(ctrl_profile, 'trackpad_for_wrist_rot', False)
        if self.trackpad_for_wrist_rot:
            # Configure trackpad rotating palm
            self.trackpad_rot_rate = ctrl_profile.trackpad_rot_rate  # How much to rotate (Rad) per second in trackpad
            self.trackpad_rot_rate = 0.2   # How much to rotate (Rad) per second in trackpad
            # Register function
            self.get_joint_fn = ProcessFn(
                "get_joint_positions_with_trackpad_rot",
                self.get_joint_fn_with_trackpad_rot,
            )

    def _init_correction_maneuver_fn(self, ctrl_profile):
        self.indicate_corrective_maneuver = getattr(ctrl_profile, "corrective_maneuver", False)
        if not self.indicate_corrective_maneuver:
            return 
        # Start ROS node of name "/vr_controller_info/corrective_maneuver" with boolean.
        # Create a processing function that if menu button is held, send 1, otherwise 0
        # Register function to self.process_fns
        self.corrective_maneuver_pub = self.node.create_publisher(
            Bool, 
            f"corrective_maneuver_{self.side.arm_name}", 
            10
        )
        def corrective_maneuver_fn(controller_info: ControllerInfo):
            """
            If the menu button is pressed, send a corrective maneuver signal.
            """
            self.corrective_maneuver_pub.publish(
                Bool(data=controller_info.menu_button)
            )
        self.process_fns['corrective_maneuver'] = ProcessFn(
            "corrective_maneuver",
            corrective_maneuver_fn,
            when_gripped=False
        )

    def _compute_target_joints_from_pose(self, controller_pose):
        """Compute target joint positions from controller pose"""
        # Transform controller pose to base frame and compute IK
        delta_in_controller = np_tf_inv(self.controller_in_vr0) @ controller_pose
        
        # Add sensitivity
        delta_in_controller[:3, 3] *= self.position_sensitivity
        rotation_mat = delta_in_controller[:3, :3]
        rotation = R.from_matrix(rotation_mat)
        # Convert to axis-angle and scale the angle
        axis, angle = rotation.as_rotvec(), np.linalg.norm(rotation.as_rotvec())
        if angle > 1e-6:
            scaled_angle = angle * self.orientation_sensitivity  # e.g., 2.0
            new_rotvec = axis / angle * scaled_angle
            new_rotation_mat = R.from_rotvec(new_rotvec).as_matrix()
            delta_in_controller[:3, :3] = new_rotation_mat

        current_state, _ = self.get_current_commanded_tool(self.tool_frame)
        target_in_base = self.tool_in_base0 @ delta_in_controller

        current_controller_in_vr_msg = TransformStamped()
        current_controller_in_vr_msg.transform = mat_to_transform(controller_pose)
        current_controller_in_vr_msg.header.frame_id = VR_FRAME_NAME
        current_controller_in_vr_msg.child_frame_id = f"controller_{self.side.arm_name}_in_vr"
        self.tf_broadcaster.sendTransform(current_controller_in_vr_msg)

        target_tool_in_base_msg = TransformStamped()
        target_tool_in_base_msg.transform = mat_to_transform(target_in_base)
        target_tool_in_base_msg.header.frame_id = self.base_frame
        target_tool_in_base_msg.child_frame_id = f"target_{self.tool_frame}"
        self.tf_broadcaster.sendTransform(target_tool_in_base_msg)

        # Solve IK
        pose_goal = Pose()
        pose_goal.position.x = target_tool_in_base_msg.transform.translation.x
        pose_goal.position.y = target_tool_in_base_msg.transform.translation.y
        pose_goal.position.z = target_tool_in_base_msg.transform.translation.z
        pose_goal.orientation = target_tool_in_base_msg.transform.rotation

        robot_state = deepcopy(current_state)
        max_retries = 3
        for i in range(max_retries):
            success = robot_state.set_from_ik(self.side.arm_name, pose_goal, self.tool_frame)
            if not success:
                continue
            q = np.asarray(robot_state.get_joint_group_positions(self.side.arm_name),
                    dtype=float)
            if np.all(np.isfinite(q)):
                return q          # <-- guaranteed no NaN/Inf
        self.node.get_logger().info("IK failed!")
        return None
    # ---------------------------
    # Custom Joint Getting Functions (with support for macros)
    # ---------------------------
    def get_joint_positions(self, controller_info: ControllerInfo):
        """
        Get joint positions from controller info. This is the method called by async processing.
        Can be overridden by custom implementations that handle button macros.
        """
        controller_pose = self.get_controller_in_vr(controller_info)
        return self._compute_target_joints_from_pose(controller_pose)
    
    def get_joint_fn_with_trackpad_rot(self, controller_info: ControllerInfo):
        """Get joint positions with trackpad rotation functionality"""
        self.last_joint_t = getattr(self, 'last_joint_t', perf_counter())
        dt = perf_counter() - self.last_joint_t
        self.last_joint_t = perf_counter()
        # If trackpad is touched, rotate the gripper relative to goal joint positions
        if self.is_trackpad_leftclick(controller_info):
            delta_rot = rotation_matrix_gripper_joint(dt * self.trackpad_rot_rate)
            self.controller_in_vr0 = self.controller_in_vr0 @ delta_rot
        elif self.is_trackpad_rightclick(controller_info):
            delta_rot = rotation_matrix_gripper_joint(dt * -self.trackpad_rot_rate)
            self.controller_in_vr0 = self.controller_in_vr0 @ delta_rot
        return self.get_joint_positions(controller_info)

    # ---------------------------
    # Global Processing Functions
    # ---------------------------
    def process_input(self, controller_info: ControllerInfo):
        """
        Process controller input
        """
        # if session is on
        for fn_name, fn in self.process_fns.items():
            if not fn.when_gripped or controller_info.grip_button:
                fn(controller_info)

    # ---------------------------
    # Button Mapping functions
    # ---------------------------
    def is_trackpad_leftclick(self, controller_info: ControllerInfo):
        return (
            controller_info.trackpad_axis_x < -self.trackpad_click_deadzone \
            and controller_info.trackpad_button
        )
    def is_trackpad_rightclick(self, controller_info: ControllerInfo):
        return (
            controller_info.trackpad_axis_x > self.trackpad_click_deadzone \
            and controller_info.trackpad_button
        )
    def is_trackpad_upclick(self, controller_info: ControllerInfo):
        return (
            controller_info.trackpad_axis_y > self.trackpad_click_deadzone \
            and controller_info.trackpad_button
        )
    def is_trackpad_downclick(self, controller_info: ControllerInfo):
        return (
            controller_info.trackpad_axis_y < -self.trackpad_click_deadzone \
            and controller_info.trackpad_button
        )

    def toggle_gripper_up_down(self, controller_info: ControllerInfo):
        if self.gripper_in_motion:
            return
        # Hack to prevent toggling too fast
        if perf_counter() - self.last_toggled_gripper < self.toggle_gripper_dt:
            return
        # Toggle with trackpad up/down clicks
        if self.is_trackpad_downclick(controller_info):
            if self.gripper_state_idx >= self._max_gripper_idx:
                print("Gripper is already at the maximum state, cannot toggle down.")
                return
            self.gripper_state_idx += 1
            self.side.set_gripper_position(self.gripper_keypoints[self.gripper_state_idx][0],
                        scissor_position=self.gripper_keypoints[self.gripper_state_idx][-1])
        elif self.is_trackpad_upclick(controller_info):
            if self.gripper_state_idx <= 0:
                print("Gripper is already at the minimum state, cannot toggle up.")
                return
            self.gripper_state_idx -= 1
            self.side.set_gripper_position(self.gripper_keypoints[self.gripper_state_idx][0],
                        scissor_position=self.gripper_keypoints[self.gripper_state_idx][-1])
        self.last_toggled_gripper = perf_counter()

    # ---------------------------
    # Robot Pose related functions
    # ---------------------------
    def on_start_recording(self, controller_info: ControllerInfo):
        controller_in_vr0 = self.get_controller_in_vr(controller_info)
        self.controller_in_vr0 = controller_in_vr0

        controller_in_vr0_msg = TransformStamped()
        controller_in_vr0_msg.transform = mat_to_transform(controller_in_vr0)
        controller_in_vr0_msg.header.frame_id = VR_FRAME_NAME
        controller_in_vr0_msg.child_frame_id = f"controller_{self.side.arm_name}_in_vr0"
        self.tf_broadcaster.sendTransform(controller_in_vr0_msg)

        _, self.tool_in_base0 = self.get_current_commanded_tool(self.tool_frame)

        tool_in_base0_msg = TransformStamped()
        tool_in_base0_msg.transform = mat_to_transform(self.tool_in_base0)
        tool_in_base0_msg.header.frame_id = self.base_frame
        tool_in_base0_msg.child_frame_id = f"tool_{self.side.arm_name}_in_base0"
        self.tf_broadcaster.sendTransform(tool_in_base0_msg)

        controller_in_vr0_msg.child_frame_id = f"controller_{self.side.arm_name}_in_vr0"
        self.tf_broadcaster.sendTransform(controller_in_vr0_msg)

    def on_stop_recording(self):
        """Stop asynchronous processing when recording stops"""
        pass

    def send_pose_cmd(self, controller_info: ControllerInfo):
        joint_positions = self.get_joint_fn(controller_info)

        if joint_positions is None:
            return

        # Use filtered positions for smoother control
        if self.use_filter:
            joint_positions_filtered = self.filter.update(joint_positions)
            joint_state_msg = JointState()
            joint_state_msg.name = [f"victor_{self.side.arm_name}_joint_{i}" for i in
                                    range(len(joint_positions_filtered))]
            joint_state_msg.position = joint_positions_filtered.squeeze().tolist()
            self.filter_pub.publish(joint_state_msg)
            joint_positions = joint_positions_filtered

        joint_positions = np.clip(joint_positions, self.side.lower, self.side.upper)
        self.side.send_joint_cmd(joint_positions)

    def get_current_commanded_tool(self, frame):
        current_state = RobotState(self.robot_model)
        motion_status: MotionStatus = self.side.motion_status.get()
        current_cmd_positions = jvq_to_list(motion_status.commanded_joint_position)
        current_state.set_joint_group_positions(self.side.arm_name, current_cmd_positions)
        current_state.update()
        tool_in_base = current_state.get_global_link_transform(frame)
        return current_state, tool_in_base

    def update(self):
        gripper_status: Robotiq3FingerStatus = self.side.gripper_status.get()
        self.gripper_in_motion = gripper_status.gripper_motion_status == Robotiq3FingerStatus.GRIPPER_IN_MOTION

class VictorTeleopNode(Node):
    def __init__(self, ctrl_profile: VictorTeleopProfile):
        super().__init__("victor_vr_teleop")

        self.victor = Victor(self)

        moveit_package_name = "victor_moveit_config"
        moveit_cpp_path = get_package_share_path(moveit_package_name) / "config" / "moveit_cpp.yaml"
        kinematics_path = "config/vr_teleop_kinematics.yaml"
        builder = MoveItConfigsBuilder(robot_name='victor', package_name=moveit_package_name)
        builder = builder.moveit_cpp(str(moveit_cpp_path))
        builder = builder.robot_description_kinematics(str(kinematics_path))
        config_dict = builder.to_moveit_configs().to_dict()

        self.moveitpy = MoveItPy(node_name="victor_vr_teleop_moveitpy", config_dict=config_dict)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parse profile
        self.ctrl_profile = ctrl_profile

        # on controllers info depends on left
        self.vr_sub = self.create_subscription(ControllersInfo, "vr_controller_info", self.on_controllers_info, 10)

        self.use_viewport_tracker = self.ctrl_profile.use_viewport_tracker

        self.has_started = False
        self.is_recording = False
        self.rcv_dts = []
        self.last_rcv_t = perf_counter()

        # Runtime initialization
        self._initialized = False
        
        # Performance monitoring
        self.slow_callback_count = 0
        self.callback_times = []
        self.last_performance_report = perf_counter()

    def _runtime_init(self):
        # Left Gripper
        self.use_left = self.ctrl_profile.use_left
        self.use_right = self.ctrl_profile.use_right
        self.use_viewport_tracker = self.ctrl_profile.use_viewport_tracker

        if self.use_viewport_tracker:
            self.viewport_tracker = ViewportTracker(
                self, 
                self.tf_broadcaster, 
                self.ctrl_profile
            )

        for side, use in zip(['left', 'right'], [self.use_left, self.use_right]):
            if not use:
                continue
            init_joints = self.ctrl_profile.init_joints.get(side, None)
            if not init_joints:
                return
            # Plan to initial joint configuration
            ctrl_setter = getattr(self.victor, f'set_{side}_controller')
            res = ctrl_setter("joint_impedance_trajectory_controller")
            self.victor.plan_to_joint_config(init_joints, f"{side}_arm")

        # Then instantiate, so controller will not be overwritten by joint_trajectory
        for side, use in zip(['left', 'right'], [self.use_left, self.use_right]):
            if not use:
                setattr(self, side, None)
                continue
            side_obj = SideTeleop(self, getattr(self.victor, side),
                self.victor,
                self.moveitpy,
                self.tf_broadcaster,
                self.ctrl_profile
            )
            setattr(self, side, side_obj)

        self._initialized = True
        print("ready!")

    def on_controllers_info(self, msg: ControllersInfo):
        if not self._initialized:
            self._runtime_init()
            return
        
        if len(msg.controllers_info) == 0:
            return
        
        if self.use_left:
            self.left.update()
        if self.use_right:
            self.right.update()
        
        any_grip_button = any([controller_info.grip_button for controller_info in msg.controllers_info])
        # any_menu_button = any([controller_info.menu_button for controller_info in msg.controllers_info])

        # viz controllers in rviz - only send TF if needed
        if hasattr(self, '_tf_counter'):
            self._tf_counter += 1
        else:
            self._tf_counter = 0
            
        # viz controllers in rviz
        vr_to_root = TransformStamped()
        vr_to_root.header.stamp = self.get_clock().now().to_msg()
        vr_to_root.header.frame_id = "victor_root"
        vr_to_root.child_frame_id = VR_FRAME_NAME
        vr_to_root.transform.translation.x = 1.5
        vr_to_root.transform.translation.z = 1.5
        q_wxyz = transforms3d.euler.euler2quat(0, 0, np.pi)
        vr_to_root.transform.rotation.w = q_wxyz[0]
        vr_to_root.transform.rotation.x = q_wxyz[1]
        vr_to_root.transform.rotation.y = q_wxyz[2]
        vr_to_root.transform.rotation.z = q_wxyz[3]

        self.tf_broadcaster.sendTransform(vr_to_root)
        for controller_info in msg.controllers_info:
            self.tf_broadcaster.sendTransform(controller_info_to_tf(self, controller_info))

        # Publish tracker TF transforms
        for tracker_info in msg.trackers_info:
            self.tf_broadcaster.sendTransform(tracker_info_to_tf(self, tracker_info))

        # Start recording
        if not self.is_recording and any_grip_button:
            self.start_recording(msg)
        elif self.is_recording and not any_grip_button:
            self.stop_recording(msg)
            
        # Process inputs for each controller AND TRACKER
        for controller_info in msg.controllers_info:
            if 'left' in controller_info.controller_name and self.use_left:
                self.left.process_input(controller_info)
            elif 'right' in controller_info.controller_name and self.use_right:
                self.right.process_input(controller_info)
        
        for tracker_info in msg.trackers_info:
            if "Head" in tracker_info.tracker_name \
                and self.use_viewport_tracker:
                self.viewport_tracker.process_input(tracker_info)
                break

        self.perf_record()
        

    def start_recording(self, msg):
        self.is_recording = True
        self.has_started = True
        self.tracker_recorded = False
                
        # Handle controller info for arms
        for controller_info in msg.controllers_info:
            if 'left' in controller_info.controller_name and self.use_left:
                self.left.on_start_recording(controller_info)
            elif 'right' in controller_info.controller_name and self.use_right:
                self.right.on_start_recording(controller_info)

    def stop_recording(self, msg):
        self.is_recording = False
        self.tracker_recorded = False
        for controller_info in msg.controllers_info:
            if 'left' in controller_info.controller_name and self.use_left:
                self.left.on_stop_recording()
            elif 'right' in controller_info.controller_name and self.use_right:
                self.right.on_stop_recording()

    def perf_record(self):
        now = perf_counter()
        rcv_dt = now - self.last_rcv_t
        self.rcv_dts.append(rcv_dt)
        if len(self.rcv_dts) > 100:
            self.rcv_dts.pop(0)
        mean_rcv_dt = np.mean(self.rcv_dts)
        # print(f"mean_rcv_dt: {mean_rcv_dt:.3f} seconds, rcv_dt: {rcv_dt:.3f} seconds")
        if mean_rcv_dt > 0.05:
            self.get_logger().warn(f'slow!!! {mean_rcv_dt=:.3f}')
        self.last_rcv_t = now

def main():
    np.seterr(all='raise')
    np.set_printoptions(precision=3, suppress=True)
    rclpy.init()

    # --- parse command‐line arguments ---
    parser = argparse.ArgumentParser(description="Launch Victor teleop with options")
    parser.add_argument(
        '--profile',
        choices=teleop_profile_dict.keys(),
        default=list(teleop_profile_dict.keys())[0],
        help="Profile for VR control"
    )
    args = parser.parse_args()
    # Control profile
    ctrl_profile_class = teleop_profile_dict.get(args.profile, None)
    if ctrl_profile_class is None:
        raise ValueError(f"Profile {args.profile} not found in teleop profiles.")
    node = VictorTeleopNode(ctrl_profile=ctrl_profile_class())

    # Use SingleThreadedExecutor instead of MultiThreadedExecutor to avoid threading issues
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except SystemExit:
        pass

    rclpy.shutdown()
    print("Done!")


if __name__ == '__main__':
    main()
