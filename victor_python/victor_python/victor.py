from typing import Sequence, Optional, Callable

import numpy as np
from transforms3d.euler import quat2euler
from urdf_parser_py.urdf import Robot as RobotURDF
from urdf_parser_py.urdf import URDF
from urdf_parser_py.xml_reflection import core

import rclpy
from arm_robots.robot import load_moveit_config
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_utilities.listener import Listener
from victor_python.victor_utils import get_control_mode_params, is_gripper_closed, get_gripper_closed_fraction_msg, \
    jvq_to_list, list_to_jvq
from victor_hardware_interfaces.msg import MotionCommand, MotionStatus, Robotiq3FingerStatus, Robotiq3FingerCommand, \
    ControlMode
from victor_hardware_interfaces.srv import SetControlMode, GetControlMode

# Suppress error messages from urdf_parser_py
core.on_error = lambda *args: None

ROBOTIQ_OPEN = 0.0
ROBOTIQ_CLOSED = 1.0
# This must match what the IIWA_LCM_BRIDGE checks for
CARTESIAN_CMD_BASE_FRAME = "base"
# This matches the URDF, but the URDF is written to match the Sunrise Workbench.
# Changing the URDF or this name will not change the frame in which Cartesian commands are interpreted.
# The only way to do that is to change the Sunrise Workbench code.
TOOL_FRAME_SUFFIX = "tool0"
CARTESIAN_CMD_FRAME_SUFFIX = "sunrise_palm_surface"


class ControlModeError(Exception):
    pass


class Side:

    def __init__(self, node: Node, name: str):
        """
        Args:
            node: rclpy node
            name: either "left" or "right"
        """
        self.node = node
        self.name = name
        self.arm_name = f"{self.name}_arm"
        self.cartesian_cmd_tool_frame = f'victor_{self.arm_name}_{CARTESIAN_CMD_FRAME_SUFFIX}'
        self.cartesian_cmd_base_frame = f'victor_{self.arm_name}_cartesian_cmd'

        # This depends on moveit being installed and importable, but not on moveit_py or on the move_group node
        self.moveit_config = load_moveit_config("victor").to_dict()
        self.urdf = URDF.from_xml_string(self.moveit_config['robot_description'])
        self.lower, self.upper = self.parser_joint_limits()

        self.pub_group = MutuallyExclusiveCallbackGroup()
        self.set_srv_group = MutuallyExclusiveCallbackGroup()
        self.get_srv_group = MutuallyExclusiveCallbackGroup()

        self.motion_command = node.create_publisher(MotionCommand, f"victor/{self.arm_name}/motion_command", 10,
                                                    callback_group=self.pub_group)
        self.gripper_command = node.create_publisher(Robotiq3FingerCommand, f"victor/{self.arm_name}/gripper_command",
                                                     10, callback_group=self.pub_group)
        self.cartesian_cmd_abc_pub = node.create_publisher(MotionStatus, f"victor_{self.arm_name}/cartesian_cmd_abc",
                                                           10)

        self.set_control_mode = node.create_client(SetControlMode, f"victor/{self.arm_name}/set_control_mode_service",
                                                   callback_group=self.set_srv_group)
        self.get_control_mode = node.create_client(GetControlMode, f"victor/{self.arm_name}/get_control_mode_service",
                                                   callback_group=self.get_srv_group)

        self.motion_status = Listener(node, MotionStatus, f"victor/{self.arm_name}/motion_status", 10)
        self.gripper_status = Listener(node, Robotiq3FingerStatus, f"victor/{self.arm_name}/gripper_status", 10)

    def open_gripper(self):
        # TODO: implementing blocking grasping
        self.gripper_command.publish(get_gripper_closed_fraction_msg(ROBOTIQ_OPEN))

    def close_gripper(self):
        # TODO: implementing blocking grasping
        self.gripper_command.publish(get_gripper_closed_fraction_msg(ROBOTIQ_CLOSED))

    def get_gripper_status(self) -> Robotiq3FingerStatus:
        return self.gripper_status.get()

    def is_gripper_closed(self):
        status: Robotiq3FingerStatus = self.get_gripper_status()
        return is_gripper_closed(status)

    def get_motion_status(self) -> MotionStatus:
        return self.motion_status.get()

    def get_arm_control_mode(self):
        control_mode_res: GetControlMode.Response = self.get_control_mode.call(GetControlMode.Request())
        return control_mode_res.active_control_mode.control_mode

    def set_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, **kwargs)
        req = SetControlMode.Request()
        req.new_control_mode = new_control_mode
        res: SetControlMode.Response = self.get_control_mode.call(req)

        if not res.success:
            print(f"Failed to switch {self.arm_name} to control mode: {control_mode}")
            print(res.message)
        return res

    def get_names_and_cmd(self):
        status: MotionStatus = self.motion_status.get()
        commanded_positions = jvq_to_list(status.commanded_joint_position)
        names = [f"victor_{self.arm_name}_joint_{i}" for i in range(1, 8)]
        return names, commanded_positions

    def send_joint_cmd(self, joint_positions):
        """
        Send a MotionCommand with joint_positions, ordered joint1 to joint 7.
        """
        if len(joint_positions) != 7:
            raise ValueError(f"joint_positions must be length 7, got {len(joint_positions)}")

        # Check against joint limits
        if np.any(joint_positions < self.lower) or np.any(joint_positions > self.upper):
            raise ValueError(f"Joint positions out of bounds: {joint_positions}")

        active_mode: ControlMode = self.get_control_mode.call(GetControlMode.Request()).active_control_mode.control_mode
        if active_mode.mode not in [ControlMode.JOINT_POSITION, ControlMode.JOINT_IMPEDANCE]:
            raise ControlModeError(f"Cannot send joint command in {active_mode} mode")

        msg = MotionCommand()
        msg.control_mode = active_mode
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_position = list_to_jvq(joint_positions)

        self.motion_command.publish(msg)

    def parser_joint_limits(self):
        lower = []
        upper = []
        for i in range(7):
            for joint in self.urdf.joints:
                if joint.name == f"victor_{self.arm_name}_joint_{i + 1}":
                    lower.append(joint.limit.lower)
                    upper.append(joint.limit.upper)
        return np.array(lower), np.array(upper)

    def send_cartesian_cmd(self, target_hand_in_root: TransformStamped):
        """
        Fills out, validates, and sends a MotionCommand in cartesian impedance mode.
        """
        active_mode: ControlMode = self.get_control_mode.call(GetControlMode.Request()).active_control_mode.control_mode
        if active_mode.mode != ControlMode.CARTESIAN_IMPEDANCE:
            raise ControlModeError(f"Cannot send cartesian command in {active_mode} mode")

        if target_hand_in_root.header.frame_id != self.cartesian_cmd_base_frame:
            raise ValueError(f"header.frame_id must be {self.cartesian_cmd_base_frame}")
        if target_hand_in_root.child_frame_id != self.cartesian_cmd_tool_frame:
            raise ValueError(f"child_frame_id must be {self.cartesian_cmd_tool_frame}")

        msg = MotionCommand()
        msg.control_mode.mode = ControlMode.CARTESIAN_IMPEDANCE
        msg.header.stamp = self.node.get_clock().now().to_msg()
        # Override the frame_id to match the base frame, since the IIWA_LCM_BRIDGE checks for this.
        msg.header.frame_id = CARTESIAN_CMD_BASE_FRAME
        msg.cartesian_pose.position.x = target_hand_in_root.transform.translation.x
        msg.cartesian_pose.position.y = target_hand_in_root.transform.translation.y
        msg.cartesian_pose.position.z = target_hand_in_root.transform.translation.z
        msg.cartesian_pose.orientation = target_hand_in_root.transform.rotation

        # FIXME: remove ths, it's just for debugging
        #  publish the commanded RPY
        q_wxyz = [msg.cartesian_pose.orientation.w, msg.cartesian_pose.orientation.x,
                  msg.cartesian_pose.orientation.y, msg.cartesian_pose.orientation.z]
        q_abc = quat2euler(q_wxyz)
        cartesian_cmd_abc_msg = MotionStatus()
        cartesian_cmd_abc_msg.header.stamp = msg.header.stamp
        cartesian_cmd_abc_msg.commanded_cartesian_pose_abc.a = q_abc[0]
        cartesian_cmd_abc_msg.commanded_cartesian_pose_abc.b = q_abc[1]
        cartesian_cmd_abc_msg.commanded_cartesian_pose_abc.c = q_abc[2]
        self.cartesian_cmd_abc_pub.publish(cartesian_cmd_abc_msg)

        self.motion_command.publish(msg)


class Victor:

    def __init__(self, node: Node, robot_description_cb: Optional[Callable[[RobotURDF], None]] = None):
        super().__init__()
        self.node = node
        self.robot_description_user_cb = robot_description_cb

        self.left = Side(node, 'left')
        self.right = Side(node, 'right')

        self.joint_states_listener = Listener(node, JointState, 'joint_states', 10)

        # Subscribe to robot description so that we can get the joints and joint limits
        # This callback will only be called once at the beginning.
        # To get the parsed URDF, either pass in a user callback or use `victor.urdf`.
        self.description_callback_group = None  # MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = node.create_subscription(String, 'robot_description', self.robot_description_callback, qos,
                                            callback_group=self.description_callback_group)

        self.urdf: Optional[RobotURDF] = None

    def wait_for_urdf(self):
        while self.urdf is None:
            rclpy.spin_once(self.node)

    def robot_description_callback(self, msg: String):
        self.urdf = URDF.from_xml_string(msg.data)
        if self.robot_description_user_cb:
            self.robot_description_user_cb(self.urdf)

    def get_joint_states(self) -> JointState:
        return self.joint_states_listener.get()

    def get_motion_statuses(self):
        return {'left': self.left.get_motion_status(), 'right': self.right.get_motion_status()}

    def get_control_modes(self):
        return {'left': self.left.get_arm_control_mode(), 'right': self.right.get_arm_control_mode()}

    def set_control_modes(self, control_mode: ControlMode, vel: float, **kwargs):
        left_res = self.left.set_arm_control_mode(control_mode, vel=vel, **kwargs)
        right_res = self.right.set_arm_control_mode(control_mode, vel=vel, **kwargs)
        return left_res, right_res

    def get_joint_positions(self, joint_names: Optional[Sequence[str]] = None):
        position_of_joint = self.get_joint_positions_dict()
        return [position_of_joint[name] for name in joint_names]

    def get_joint_positions_dict(self):
        joint_state = self.get_joint_states()
        joint_positions = dict(zip(joint_state.name, joint_state.position))
        return joint_positions

    def get_joint_cmd_dict(self):
        left_names, left_commanded_positions = self.left.get_names_and_cmd()
        right_names, right_commanded_positions = self.right.get_names_and_cmd()

        joint_positions = dict(zip(left_names + right_names, left_commanded_positions + right_commanded_positions))
        return joint_positions
