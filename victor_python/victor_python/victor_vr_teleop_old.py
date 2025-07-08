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

import numpy as np
import transforms3d
from scipy.spatial.transform import Rotation as R

import rclpy
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
from victor_hardware_interfaces.msg import MotionStatus, ControlMode, Robotiq3FingerStatus
from victor_python.victor import Victor, Side
from victor_python.victor_utils import get_control_mode_params
from victor_python.victor_utils import get_gripper_closed_fraction_msg, jvq_to_list
from vr_ros2_bridge_msgs.msg import ControllersInfo, ControllerInfo

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


def do_nothing_fn(*args, **kwargs):
    """
    A placeholder function that does nothing.
    This is used to bind buttons to no action.
    """
    pass

class SideTeleop:
    def __init__(self, 
            node: Node, 
            side: Side, 
            moveitpy: MoveItPy, 
            tf_broadcaster: TransformBroadcaster,
            controller_usability_rotation=np.eye(3),
            position_sensitivity=1.0,
            orientation_sensitivity=1.0,
            split_pos_rot=False,
            trackpad_wrist_rot=False,
            trackpad_click_angle=np.deg2rad(30.0),  # 15° per click
            trackpad_click_deadzone=0.5,            # region threshold
            use_filter=True,
            init_gripper_state=0,                   # Initial gripper state index
            ctrl_mode="natural",              # 'natural' or 'trackpad_rot'
        ):
        """

        Args:
            node: ROS node
            side: Side object, from Victor
            moveitpy: MoveItPy object, there must only be one of there per python script/process!
            tf_broadcaster: Publishing TF for visualization
            controller_usability_rotation: Offset to the VR controller, to make the orientation more unintuitive.

        """
        self.node = node
        self.side = side
        self.moveitpy = moveitpy
        self.robot_model: RobotModel = self.moveitpy.get_robot_model()
        self.tf_broadcaster = tf_broadcaster
        self.controller_usability_rotation = controller_usability_rotation
        # self.gripper_open = True

        # Keypoints for gripper control
        self.scissor_position = 1.0
        self.finger_kpts = [0.0, 0.37, 0.51, 1.0]
        self.scissor_position_kpt = [0.5, 1.0, 1.0, 0.5]
        self.curr_gripper_position = 0

        self.last_send_open_fraction = 0.0
        self.gripper_in_motion = None

        self.controller_in_vr0 = np.eye(4)
        self.tool_in_base0 = np.eye(4)
        self.wrist_in_base0 = np.eye(4)

        self.base_frame = self.robot_model.model_frame
        self.jmg: JointModelGroup = self.robot_model.get_joint_model_group(self.side.arm_name)

        # Set both frames, to make it easier to configure modes
        assert ctrl_mode in ['natural', 'trackpad_rot'], f"Invalid control mode: {ctrl_mode}"
        self.ctrl_mode = ctrl_mode
        # if self.ctrl_mode == 'trackpad_rot':
        #     self.tool_frame = [n for n in self.jmg.link_model_names if 'link' in n][-1]
        # elif self.ctrl_mode == 'natural':
        #     self.tool_frame = self.jmg.eef_name

        self.tool_frame = self.jmg.eef_name

        # Filtering motion
        self.use_filter = use_filter
        self.filter_pub = self.node.create_publisher(JointState, 'filtered_joint_states', 10)
        self.filter = BatchOnlineFilter(numtaps=20, cutoff=0.1)

        # Set controller
        active_controller_names = self.side.get_active_controller_names()
        if len(active_controller_names) != 1:
            raise ValueError(f"Expected exactly one active controller, got {active_controller_names}")
        active_controller_name = active_controller_names[0]
        self.joint_cmd_pub = self.side.get_joint_cmd_pub(active_controller_name)

        # Sensitivity setup
        self.position_sensitivity = position_sensitivity  # e.g., 1.0
        self.orientation_sensitivity = orientation_sensitivity  # e.g., 1.0
        self.split_pos_rot = split_pos_rot  # If True, position and rotation are controlled separately through trigger button

        # Set controller button mapping processors
        self.process_fns = {"send_pose_cmd": self.send_pose_cmd,}
        self.all_time_process_fns = {}

        # Configure binding functions for buttons
        # if not split_pos_rot:
        #     self.all_time_process_fns['trigger_button_gripper_position'] = self.set_gripper_position
        
        # Trackpad button to toggle gripper
        # how much to turn per left/right click
        self.trackpad_click_angle = trackpad_click_angle
        # if |x| or |y| < deadzone we ignore it for region tests
        self.trackpad_click_deadzone = trackpad_click_deadzone
        # for edge-detection of a new click press
        self._prev_trackpad_click = False

        # Trackpad up down for gripper open/close
        self.toggle_gripper_dt = 0.25  # seconds
        self.last_toggled_gripper = perf_counter()
        self.all_time_process_fns['trackpad_updown_gripper'] = self.toggle_gripper_up_down
        self.gripper_state_idx = init_gripper_state
        self.side.set_gripper_position(self.finger_kpts[self.gripper_state_idx],
                                       scissor_position=self.scissor_position_kpt[self.gripper_state_idx])
        # self.side.open_gripper(scissor_position=self.side.gripper_open_fraction)

        if not self.split_pos_rot:
            # Offset to the held tool rotation center from the gripper joint
            # self.held_tool_pos_offset = np.array([0.0, 0.119126, 0.1397])
            self.held_tool_pos_offset = np.array([0.0, 0.36, -0.1016])
            # self.held_tool_rotation_rate = np.array([0.05, 0.0, 0.0])
            self.held_tool_rotation_rate = np.array([0.0, 0.0, 0.15])
        
        # Configure trackpad rotating palm
        self.trackpad_rot_rate = 0.2   # How much to rotate (Rad) per second in trackpad

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

        self.last_sent_pose = perf_counter()

    def on_stop_recording(self):
        self.trackpad_history = []
        pass


    def rotation_matrix_gripper_joint(self, x):
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
    
    @staticmethod
    def _Rx(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[1, 0,  0],
                         [0, c, -s],
                         [0, s,  c]])

    @staticmethod
    def _Ry(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[ c, 0, s],
                         [ 0, 1, 0],
                         [-s, 0, c]])

    @staticmethod
    def _Rz(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0],
                         [s,  c, 0],
                         [0,  0, 1]])

    def rotation_matrix_held_tool_joint(self, rate: np.ndarray):
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
        R = (self._Rz(dyaw)           # yaw about Z
             @ self._Ry(dpitch)       # pitch about Y
             @ self._Rx(droll))       # roll about X

        # 2. Express rotation about an arbitrary point p:
        p = self.held_tool_pos_offset.reshape(3, 1)
        t = (np.eye(3) - R) @ p       # == p - R p

        # 3. Pack into a 4×4 homogeneous matrix.
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = t.ravel()
        return T

    
    def process_input(self, controller_info: ControllerInfo):
        """
        Process controller input
        """
        # if session is on
        if controller_info.grip_button:
            for fn_name, fn in self.process_fns.items():
                fn(controller_info)
        for fn_name, fn in self.all_time_process_fns.items():
            fn(controller_info)

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

    def send_pose_cmd(self, controller_info: ControllerInfo):
        # Deal with rotation
        dt = perf_counter() - self.last_sent_pose
        self.last_sent_pose = perf_counter()

        # Process custom rotation macros

        # If trackpad is touched, rotate the gripper relative to goal joint positions
        if self.is_trackpad_leftclick(controller_info) and self.ctrl_mode == 'trackpad_rot':
            # If both trackpad and trigger button pressed, rotate along tool
            if controller_info.trigger_button and not self.split_pos_rot:
                delta_rot = self.rotation_matrix_held_tool_joint(dt*self.held_tool_rotation_rate)
            else:
                delta_rot = self.rotation_matrix_gripper_joint(dt * self.trackpad_rot_rate)
            self.controller_in_vr0 = self.controller_in_vr0 @ delta_rot
        elif self.is_trackpad_rightclick(controller_info) and self.ctrl_mode == 'trackpad_rot':
            if controller_info.trigger_button and not self.split_pos_rot:
                delta_rot = self.rotation_matrix_held_tool_joint(dt*-self.held_tool_rotation_rate)
            else:
                delta_rot = self.rotation_matrix_gripper_joint(dt * -self.trackpad_rot_rate)
            self.controller_in_vr0 = self.controller_in_vr0 @ delta_rot

        joint_positions = self.get_target_in_base(controller_info)
        if joint_positions is None:
            return {'joint_positions': joint_positions}

        # Use filtered positions for smoother control
        if self.use_filter:
            joint_positions_filtered = self.filter.update(joint_positions)
            joint_state_msg = JointState()
            joint_state_msg.name = [f"victor_{self.side.arm_name}_joint_{i}" for i in
                                    range(len(joint_positions_filtered))]
            joint_state_msg.position = joint_positions_filtered.squeeze().tolist()
            self.filter_pub.publish(joint_state_msg)
        
        joint_positions = joint_positions_filtered if self.use_filter else joint_positions
        joint_positions = np.clip(joint_positions, self.side.lower, self.side.upper)
        self.side.send_joint_cmd(joint_positions)

        return {'joint_positions': joint_positions,}

    def set_gripper_position(self, controller_info: ControllerInfo):
        """
        Set the open fraction of the gripper.
        """
        open_fraction = controller_info.trigger_axis
        if abs(open_fraction - self.last_send_open_fraction) > 0.05:
            # FIXME: make scissor controllable?
            self.side.gripper_command.publish(
                get_gripper_closed_fraction_msg(open_fraction, scissor_position=self.scissor_position))
            self.last_send_open_fraction = open_fraction
        return open_fraction
    
    def toggle_gripper_up_down(self, controller_info: ControllerInfo):
        if self.gripper_in_motion:
            return
        # Hack to prevent toggling too fast
        if perf_counter() - self.last_toggled_gripper < self.toggle_gripper_dt:
            return
        # Toggle with trackpad up/down clicks
        if self.is_trackpad_downclick(controller_info) \
            and self.gripper_state_idx < len(self.finger_kpts) - 1:
            self.gripper_state_idx += 1
            self.side.set_gripper_position(self.finger_kpts[self.gripper_state_idx],
                                           scissor_position=self.scissor_position_kpt[self.gripper_state_idx])
            # self.gripper_open = not self.gripper_open
        elif self.is_trackpad_upclick(controller_info) \
            and self.gripper_state_idx > 0:
            self.gripper_state_idx -= 1
            self.side.set_gripper_position(self.finger_kpts[self.gripper_state_idx],    
                                           scissor_position=self.scissor_position_kpt[self.gripper_state_idx])
        self.last_toggled_gripper = perf_counter()

    def get_target_in_base(self, controller_info: ControllerInfo) -> TransformStamped:
        current_controller_in_vr = self.get_controller_in_vr(controller_info)
        delta_in_controller = np_tf_inv(self.controller_in_vr0) @ current_controller_in_vr

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

        current_state, _ = self.get_current_commanded_tool(self.tool_frame)     # frame does not matter here

        target_in_base = self.tool_in_base0 @ delta_in_controller

        current_controller_in_vr_msg = TransformStamped()
        current_controller_in_vr_msg.transform = mat_to_transform(current_controller_in_vr)
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

        # ik_t0 = perf_counter()
        success = False
        robot_state = deepcopy(current_state)
        while True:
            ok = robot_state.set_from_ik(self.side.arm_name, pose_goal, self.tool_frame)
            if ok:
                success = True
                break
            else:
                break

        # ik_t1 = perf_counter()
        # print(f"IK took {ik_t1 - ik_t0:.3f} seconds")

        if success:
            joint_positions = robot_state.get_joint_group_positions(self.side.arm_name)
            return joint_positions
        else:
            self.node.get_logger().info("IK failed!")
            return None

    def get_current_commanded_tool(self, frame):
        current_state = RobotState(self.robot_model)
        motion_status: MotionStatus = self.side.motion_status.get()
        current_cmd_positions = jvq_to_list(motion_status.commanded_joint_position)
        current_state.set_joint_group_positions(self.side.arm_name, current_cmd_positions)
        current_state.update()
        tool_in_base = current_state.get_global_link_transform(frame)
        return current_state, tool_in_base

    def get_controller_in_vr(self, controller_info):
        controller_in_vr_msg = controller_info_to_tf(self.node, controller_info)
        controller_in_vr = transform_to_mat(controller_in_vr_msg.transform)

        usability_transform = np.eye(4)
        usability_transform[:3, :3] = self.controller_usability_rotation
        controller_in_vr_usable = controller_in_vr @ usability_transform

        return controller_in_vr_usable

    def update(self):
        gripper_status: Robotiq3FingerStatus = self.side.gripper_status.get()
        self.gripper_in_motion = gripper_status.gripper_motion_status == Robotiq3FingerStatus.GRIPPER_IN_MOTION


class VictorTeleopNode(Node):
    def __init__(self,
            use_left=True,
            use_right=True,
            left_usability_rotation=[0.0,0.0,0.0],
            right_usability_rotation=[0.0,0.0,0.0],
            # motion='relative',
            position_sensitivity=1.0,
            orientation_sensitivity=1.0,
            use_filter=True,
            split_pos_rot=False,
            trackpad_wrist_rot=False,
            init_left_joints=None,
            init_right_joints=None,
            init_gripper_state=0,
            ctrl_mode="natural",  # 'natural' or 'trackpad_rot'
            target_controller="impedance_controller"):
        super().__init__("victor_vr_teleop")

        self.victor = Victor(self)

        moveit_package_name = f"victor_moveit_config"
        moveit_cpp_path = get_package_share_path(moveit_package_name) / "config" / "moveit_cpp.yaml"
        kinematics_path = "config/vr_teleop_kinematics.yaml"
        builder = MoveItConfigsBuilder(robot_name='victor', package_name=moveit_package_name)
        builder = builder.moveit_cpp(str(moveit_cpp_path))
        builder = builder.robot_description_kinematics(str(kinematics_path))
        config_dict = builder.to_moveit_configs().to_dict()

        self.moveitpy = MoveItPy(node_name="victor_vr_teleop_moveitpy", config_dict=config_dict)
        # self.latest_action_dict = None
        self.tf_broadcaster = TransformBroadcaster(self)

        self.use_left = use_left
        self.use_right = use_right
        # self.motion_target = motion
        self.target_controller = target_controller

        # Move to predefined position
        if init_left_joints is not None and self.use_left:
            res = self.victor.set_left_controller("joint_impedance_trajectory_controller")
            self.victor.plan_to_joint_config(init_left_joints, "left_arm")
        if init_right_joints is not None and self.use_right:
            res = self.victor.set_right_controller("joint_impedance_trajectory_controller")
            self.victor.plan_to_joint_config(init_right_joints, "right_arm")

        # Setup 
        if self.use_left:
            activ_ctrl = list(set(self.victor.left.get_active_controller_names()))
            if f"left_arm_{self.target_controller}" not in activ_ctrl:
                self.victor.set_left_controller(self.target_controller)
            self.left = SideTeleop(self, self.victor.left, self.moveitpy, self.tf_broadcaster,
                                    controller_usability_rotation=transforms3d.euler.euler2mat(*left_usability_rotation),
                                    position_sensitivity=position_sensitivity,
                                    orientation_sensitivity=orientation_sensitivity,
                                    split_pos_rot=split_pos_rot,
                                    trackpad_wrist_rot=trackpad_wrist_rot,
                                    use_filter=use_filter,
                                    init_gripper_state=init_gripper_state,
                                    ctrl_mode=ctrl_mode)
        if self.use_right:
            activ_ctrl = list(set(self.victor.right.get_active_controller_names()))
            if f"right_arm_{self.target_controller}" not in activ_ctrl:
                self.victor.set_right_controller(self.target_controller)
            self.right = SideTeleop(self, self.victor.right, self.moveitpy, self.tf_broadcaster,
                                    controller_usability_rotation=transforms3d.euler.euler2mat(*right_usability_rotation),
                                    position_sensitivity=position_sensitivity,
                                    orientation_sensitivity=orientation_sensitivity,
                                    split_pos_rot=split_pos_rot,
                                    trackpad_wrist_rot=trackpad_wrist_rot,
                                    use_filter=use_filter,
                                    init_gripper_state=init_gripper_state,
                                    ctrl_mode=ctrl_mode)

        # on controllers info depends on left
        self.vr_sub = self.create_subscription(ControllersInfo, "vr_controller_info", self.on_controllers_info, 10)

        self.has_started = False
        self.is_recording = False
        self.is_done = False

        self.rcv_dts = []
        self.last_rcv_t = perf_counter()

        # Variables for absolute motion mode
        self.left_controller_onstart_pose = None
        self.right_controller_onstart_pose = None
        self.left_arm_onstart_pose = None
        self.right_arm_onstart_pose = None

        # Moved to above
        # self.set_control_modes_async()
        
    # def set_control_modes_async(self):
    #     thread = Thread(target=self.set_control_modes)
    #     thread.start()

    # def set_control_modes(self):
    #     # Call switch_controllers
    #     self.victor.deactivate_all_controllers()
    #     self.victor.activate_controllers(['left_arm_impedance_controller', 'right_arm_impedance_controller'])

    #     # Update ros params for that controller
    #     req.new_control_mode = get_control_mode_params(ControlMode.JOINT_IMPEDANCE, vel=1.0, accel=0.1)

    #     if self.use_left:
    #         self.victor.left.set_control_mode_client.call(req)
    #     if self.use_right:
    #         self.victor.right.set_control_mode_client.call(req)

    def on_controllers_info(self, msg: ControllersInfo):
        if len(msg.controllers_info) == 0:
            return

        if self.use_left:
            self.left.update()
        if self.use_right:
            self.right.update()

        any_grip_button = any([controller_info.grip_button for controller_info in msg.controllers_info])
        # print("Grip Button Pressed:", any_grip_button)
        any_menu_button = any([controller_info.menu_button for controller_info in msg.controllers_info])

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

        # Start recording
        if not self.is_recording and any_grip_button:
            self.is_recording = True
            self.has_started = True
            for controller_info in msg.controllers_info:
                if 'left' in controller_info.controller_name and self.use_left:
                    self.left.on_start_recording(controller_info)
                elif 'right' in controller_info.controller_name and self.use_right:
                    self.right.on_start_recording(controller_info)

        # Stop recording
        elif self.is_recording and not any_grip_button:
            self.is_recording = False
            for controller_info in msg.controllers_info:
                if 'left' in controller_info.controller_name and self.use_left:
                    self.left.on_stop_recording()
                elif 'right' in controller_info.controller_name and self.use_right:
                    self.right.on_stop_recording()

        if self.has_started and any_menu_button:
            self.is_done = True
            self.on_done()

        controller_info: ControllerInfo
        for controller_info in msg.controllers_info:
            if 'left' in controller_info.controller_name and self.use_left:
                self.left.process_input(controller_info)
            elif 'right' in controller_info.controller_name and self.use_right:
                self.right.process_input(controller_info)

        now = perf_counter()
        rcv_dt = self.last_rcv_t - now
        self.rcv_dts.append(rcv_dt)
        if len(self.rcv_dts) > 100:
            self.rcv_dts.pop(0)
        mean_rcv_dt = np.mean(self.rcv_dts)
        if mean_rcv_dt > 0.038:
            print(f'slow!!! {mean_rcv_dt=:.3f}')

        self.last_rcv_t = now

    def on_done(self):
        self.victor.left.open_gripper()
        self.victor.right.open_gripper()

        raise SystemExit("Done!")


def main():

    np.seterr(all='raise')
    np.set_printoptions(precision=3, suppress=True)

    rclpy.init()

    # --- parse command‐line arguments ---
    parser = argparse.ArgumentParser(description="Launch Victor teleop with options")
    parser.add_argument(
        '--arms',
        choices=['left', 'right', 'both'],
        default='both',
        help="Which arm(s) to control"
    )
    parser.add_argument(
        '--position_sensitivity',
        type=float,
        default=1.0,
        help="Scaling factor for positional motion"
    )
    parser.add_argument(
        '--orientation_sensitivity',
        type=float,
        default=1.0,
        help="Scaling factor for rotational motion"
    )
    parser.add_argument(
        '--target_controller',
        type=str,
        default="impedance_controller",
        choices=["impedance_controller", "position_controller"],
        help="Name of the target controller to switch to"
    )
    parser.add_argument(
        '--use_filter',
        action='store_true',
        help="Enable filtering of controller inputs"
    )
    parser.add_argument(
        '--split_pos_rot',
        action='store_true',
        help="Split position and rotation control through trigger button"
    )
    parser.add_argument(
        '--trackpad_wrist_rot',
        action='store_true',
        help="Use trackpad to rotate the wrist of the arm instead of rotating controller"
    )
    parser.add_argument(
        '--init_joints',
        action='store_true',
        help="Initialize the arm to a predefined joint configuration defined in the script"
    )
    parser.add_argument(
        '--init_gripper_state',
        type=int,
        default=0,
        help="Initialize the gripper to predefined state configured in Side class"
    )
    parser.add_argument(
        '--ctrl_mode',
        default='natural',
        choices=['natural', 'trackpad_rot'],
        help="Control mode for the arm, either 'natural' or 'trackpad_rot'. "
    )
    args = parser.parse_args()

    # --- translate args.arms into boolean flags ---
    use_left = args.arms in ('left', 'both')
    use_right = args.arms in ('right', 'both')

    # roll, pitch, yaw
    # roll = pi is setting gripper face down
    usability_rotation = [np.pi, 0.0, 0.0]

    # Engine on Crater configuration
    # init_left_joints = [-0.63023839,
    #     1.1021754,
    #     -0.7190757,
    #     1.423316,
    #     -0.4328417,
    #     -0.91821772,
    #     -2.6
    # ]
    # Engine on mount configuration
    init_left_joints = [
        1.1847989247915828,
        -0.46831252260769395,
        -0.9837651464937674,
        -1.6244355817879254,
        0.4139760770596477,
        1.0696352634055712,
        1.287291193753409
    ]

    # Engine on Crater configuration
    # init_right_joints = [
    #     0.6092781913562261,
    #     1.108405883333034,
    #     0.3733802278333334,
    #     1.3890371957842383,
    #     0.8443950751167238,
    #     -1.0637165864373954,
    #     1.9409640643569282
    # ]
    # Engine on mount configuration
    init_right_joints = [1.706375053909089,
        0.5047621455575189,
        -1.6661934509388395,
        -1.0337042757247657,
        -0.41568294934346726,
        1.4536155554562367,
        0.4079876871561164
    ]


    # --- instantiate your node with the chosen options ---
    node = VictorTeleopNode(
        use_left=use_left,
        use_right=use_right,
        left_usability_rotation=usability_rotation,
        right_usability_rotation=usability_rotation,
        position_sensitivity=args.position_sensitivity,
        orientation_sensitivity=args.orientation_sensitivity,
        use_filter=args.use_filter,
        split_pos_rot=args.split_pos_rot,
        trackpad_wrist_rot=args.trackpad_wrist_rot,
        init_left_joints=init_left_joints if args.init_joints else None,
        init_right_joints=init_right_joints if args.init_joints else None,
        init_gripper_state=args.init_gripper_state,
        ctrl_mode=args.ctrl_mode,
        target_controller=args.target_controller
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    print("ready!")

    try:
        executor.spin()
    except SystemExit:
        pass

    rclpy.shutdown()
    print("Done!")


if __name__ == '__main__':
    main()
