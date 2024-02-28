from typing import Sequence, Optional, Callable

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import URDF, Robot

# Suppress error messages from urdf_parser_py
from urdf_parser_py.xml_reflection import core

core.on_error = lambda *args: None

from arm_utilities.listener import Listener
from victor_hardware.victor_utils import get_control_mode_params, is_gripper_closed, get_gripper_closed_fraction_msg, \
    jvq_to_list
from victor_hardware_interfaces.msg import MotionCommand, MotionStatus, Robotiq3FingerStatus, Robotiq3FingerCommand, \
    ControlMode
from victor_hardware_interfaces.srv import SetControlMode, GetControlMode

ROBOTIQ_OPEN = 0.0
ROBOTIQ_CLOSED = 1.0


class Side:

    def __init__(self, node: Node, name: str):
        """

        Args:
            name: either "left" or "right"
        """
        self.node = node
        self.name = name
        self.arm_name = f"{self.name}_arm"

        self.pub_group = MutuallyExclusiveCallbackGroup()
        self.set_srv_group = MutuallyExclusiveCallbackGroup()
        self.get_srv_group = MutuallyExclusiveCallbackGroup()

        self.motion_command = node.create_publisher(MotionCommand, f"victor/{self.arm_name}/motion_command", 10,
                                                    callback_group=self.pub_group)
        self.gripper_command = node.create_publisher(Robotiq3FingerCommand, f"victor/{self.arm_name}/gripper_command",
                                                     10, callback_group=self.pub_group)

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


# TODO: inherit from Robot in arm_robots.robot
class Victor:
    def __init__(self, node: Node, robot_description_cb: Optional[Callable[[Robot], None]] = None):
        self.node = node
        self.robot_description_user_cb = robot_description_cb

        self.left = Side(node, 'left')
        self.right = Side(node, 'right')

        self.joint_states_listener = Listener(node, JointState, 'joint_states', 10)

        # Subscribe to robot description we can get the joints and joint limits
        # This callback will only be called once at the beginning.
        # To get the parsed URDF, either pass in a user callback or use `victor.urdf`.
        self.description_callback_group = None  # MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = node.create_subscription(String, 'robot_description', self.robot_description_callback, qos,
                                            callback_group=self.description_callback_group)

        self.urdf: Optional[Robot] = None

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
