from dataclasses import dataclass
from typing import Sequence, Optional

from arm_utilities.listener import Listener
from urdf_parser_py.urdf import URDF, Robot
from victor_hardware_interfaces.msg import MotionCommand, MotionStatus, Robotiq3FingerStatus, Robotiq3FingerCommand, \
    ControlMode
from victor_hardware_interfaces.srv import SetControlMode, GetControlMode

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from victor_hardware.victor_utils import get_control_mode_params, is_gripper_closed, get_gripper_closed_fraction_msg

ROBOTIQ_OPEN = 0.0
ROBOTIQ_CLOSED = 1.0


# from arm_robots.robot import Robot

@dataclass
class Side:
    name: str
    motion_command: rclpy.node.Publisher
    gripper_command: rclpy.node.Publisher
    motion_status: Listener
    gripper_status: Listener
    set_control_mode: rclpy.node.Client
    get_control_mode: rclpy.node.Client


# TODO: inherit from Robot in arm_robots.robot
class Victor:
    def __init__(self, node: Node):
        self.node = node

        self.pub_group = MutuallyExclusiveCallbackGroup()
        self.srv_group = MutuallyExclusiveCallbackGroup()
        self.listener_group = MutuallyExclusiveCallbackGroup()

        self.left_arm_cmd_pub = node.create_publisher(MotionCommand, "victor/left_arm/motion_command", 10,
                                                      callback_group=self.pub_group)
        self.right_arm_cmd_pub = node.create_publisher(MotionCommand, "victor/right_arm/motion_command", 10,
                                                       callback_group=self.pub_group)
        self.left_gripper_cmd_pub = node.create_publisher(Robotiq3FingerCommand, "victor/left_arm/gripper_command", 10,
                                                          callback_group=self.pub_group)
        self.right_gripper_cmd_pub = node.create_publisher(Robotiq3FingerCommand, "victor/right_arm/gripper_command",
                                                           10, callback_group=self.pub_group)

        self.left_set_control_mode_srv = node.create_client(SetControlMode, "victor/left_arm/set_control_mode_service",
                                                            callback_group=self.srv_group)
        self.right_set_control_mode_srv = node.create_client(SetControlMode,
                                                             "victor/right_arm/set_control_mode_service",
                                                             callback_group=self.srv_group)
        self.left_get_control_mode_srv = node.create_client(GetControlMode, "victor/left_arm/get_control_mode_service",
                                                            callback_group=self.srv_group)
        self.right_get_control_mode_srv = node.create_client(GetControlMode,
                                                             "victor/right_arm/get_control_mode_service",
                                                             callback_group=self.srv_group)

        self.joint_states_listener = Listener(node, JointState, "/victor/joint_states", 10,
                                              callback_group=self.listener_group)
        self.left_arm_status_listener = Listener(node, MotionStatus, "victor/left_arm/motion_status", 10,
                                                 callback_group=self.listener_group)
        self.right_arm_status_listener = Listener(node, MotionStatus, "victor/right_arm/motion_status", 10,
                                                  callback_group=self.listener_group)
        self.left_gripper_status_listener = Listener(node, Robotiq3FingerStatus, "victor/left_arm/gripper_status", 10,
                                                     callback_group=self.listener_group)
        self.right_gripper_status_listener = Listener(node, Robotiq3FingerStatus, "victor/right_arm/gripper_status", 10,
                                                      callback_group=self.listener_group)

        self.left = Side('left', self.left_arm_cmd_pub, self.left_gripper_cmd_pub, self.left_arm_status_listener,
                         self.left_gripper_status_listener, self.left_set_control_mode_srv,
                         self.left_get_control_mode_srv)
        self.right = Side('right', self.right_arm_cmd_pub, self.right_gripper_cmd_pub, self.right_arm_status_listener,
                          self.right_gripper_status_listener, self.right_set_control_mode_srv,
                          self.right_get_control_mode_srv)

        # subscribe to robot description we can get the joints and joint limits
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = node.create_subscription(String, '/victor/robot_description', self.robot_description_callback, qos)

        self.urdf: Optional[Robot] = None

    def wait_for_urdf(self):
        while self.urdf is None:
            rclpy.spin_once(self.node)

    def robot_description_callback(self, msg: String):
        self.urdf = URDF.from_xml_string(msg.data)

    def open_left_gripper(self):
        self.left_gripper_cmd_pub.publish(get_gripper_closed_fraction_msg(ROBOTIQ_OPEN))

    def open_right_gripper(self):
        self.right_gripper_cmd_pub.publish(get_gripper_closed_fraction_msg(ROBOTIQ_OPEN))

    def close_left_gripper(self):
        # TODO: implementing blocking grasping
        self.left_gripper_cmd_pub.publish(get_gripper_closed_fraction_msg(ROBOTIQ_CLOSED))

    def close_right_gripper(self):
        self.right_gripper_cmd_pub.publish(get_gripper_closed_fraction_msg(ROBOTIQ_CLOSED))

    def get_joint_states(self) -> JointState:
        return self.joint_states_listener.get()

    def get_right_gripper_status(self) -> Robotiq3FingerStatus:
        return self.right_gripper_status_listener.get()

    def get_left_gripper_status(self) -> Robotiq3FingerStatus:
        return self.left_gripper_status_listener.get()

    def is_left_gripper_closed(self):
        return self.is_gripper_closed('left')

    def is_right_gripper_closed(self):
        return self.is_gripper_closed('right')

    def is_gripper_closed(self, gripper: str):
        if gripper == 'left':
            status = self.get_left_gripper_status()
        elif gripper == 'right':
            status = self.get_right_gripper_status()
        else:
            raise NotImplementedError(f"invalid gripper {gripper}")
        return is_gripper_closed(status)

    def get_arms_statuses(self):
        return {'left': self.get_left_arm_status(),
                'right': self.get_right_arm_status()}

    def get_right_arm_status(self) -> MotionStatus:
        return self.right_arm_status_listener.get()

    def get_left_arm_status(self) -> MotionStatus:
        return self.left_arm_status_listener.get()

    def get_control_modes(self):
        return {'left': self.get_left_arm_control_mode(), 'right': self.get_right_arm_control_mode()}

    def set_control_modes(self, control_mode: ControlMode, vel: float, **kwargs):
        left_res = self.set_left_arm_control_mode(control_mode, vel=vel, **kwargs)
        right_res = self.set_right_arm_control_mode(control_mode, vel=vel, **kwargs)
        return left_res, right_res

    def get_left_arm_control_mode(self):
        left_control_mode_res: GetControlMode.Response = self.left_get_control_mode_srv.call(GetControlMode.Request())
        return left_control_mode_res.active_control_mode.control_mode

    def get_right_arm_control_mode(self):
        right_control_mode_res: GetControlMode.Response = self.right_get_control_mode_srv.call(GetControlMode.Request())
        return right_control_mode_res.active_control_mode.control_mode

    def set_right_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        return self.set_control_mode(self.right_set_control_mode_srv, 'right', control_mode, **kwargs)

    def set_left_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        return self.set_control_mode(self.left_set_control_mode_srv, 'left', control_mode, **kwargs)

    def set_control_mode(self, service_server, name: str, control_mode: ControlMode, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, **kwargs)
        req = SetControlMode.Request()
        req.new_control_mode = new_control_mode
        res: SetControlMode.Response = service_server.call(req)

        if not res.success:
            print(f"Failed to switch {name} to control mode: {control_mode}")
            print(res.message)
        return res

    def get_joint_positions(self, joint_names: Optional[Sequence[str]] = None):
        position_of_joint = self.get_joint_positions_dict()
        return [position_of_joint[name] for name in joint_names]

    def get_joint_positions_dict(self):
        joint_state = self.get_joint_states()
        joint_positions = dict(zip(joint_state.name, joint_state.position))
        return joint_positions
