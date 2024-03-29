from typing import Sequence, Optional, Callable, List

import numpy as np
import rclpy
from arm_utilities.ros_helpers import wait_for_subscriber
from arm_utilities.listener import Listener
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from urdf_parser_py.urdf import Robot as RobotURDF
from urdf_parser_py.urdf import URDF
from urdf_parser_py.xml_reflection import core

from arm_robots.robot import load_moveit_config
from victor_hardware_interfaces.msg import MotionStatus, Robotiq3FingerStatus, Robotiq3FingerCommand, \
    ControlModeParameters, ControlMode
from victor_python.robotiq_finger_angles import compute_finger_angles, get_finger_angle_names, compute_scissor_angle, \
    get_scissor_joint_name
from victor_python.victor_utils import is_gripper_closed, get_gripper_closed_fraction_msg, jvq_to_list

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
        self.cm_srv_group = MutuallyExclusiveCallbackGroup()

        self.cartesian_cmd_pub = node.create_publisher(Pose, f"{self.arm_name}_cartesian_controller", 10,
                                                       callback_group=self.pub_group)
        self.gripper_command = node.create_publisher(Robotiq3FingerCommand, f"victor/{self.arm_name}/gripper_command",
                                                     10, callback_group=self.pub_group)
        self.list_controllers_client = node.create_client(ListControllers, f"controller_manager/list_controllers",
                                                          callback_group=self.cm_srv_group)

        self.motion_status = Listener(node, MotionStatus, f"victor/{self.arm_name}/motion_status", 10)
        self.gripper_status = Listener(node, Robotiq3FingerStatus, f"victor/{self.arm_name}/gripper_status", 10)
        self.control_mode_listener = Listener(node, ControlModeParameters,
                                              f"victor/{self.arm_name}/control_mode_parameters", 10)

        self.controller_publishers: dict[str, Publisher] = {}

    def open_gripper(self, scissor_position=0.5):
        # TODO: implementing blocking grasping
        self.gripper_command.publish(get_gripper_closed_fraction_msg(ROBOTIQ_OPEN, scissor_position))

    def close_gripper(self, scissor_position=0.5):
        # TODO: implementing blocking grasping
        self.gripper_command.publish(get_gripper_closed_fraction_msg(ROBOTIQ_CLOSED, scissor_position))

    def get_gripper_status(self) -> Robotiq3FingerStatus:
        return self.gripper_status.get()

    def is_gripper_closed(self):
        status: Robotiq3FingerStatus = self.get_gripper_status()
        return is_gripper_closed(status)

    def get_motion_status(self) -> MotionStatus:
        return self.motion_status.get()

    def get_arm_control_mode(self):
        control_mode_res: ControlModeParameters = self.control_mode_listener.get()
        return control_mode_res.control_mode

    def get_names_and_cmd(self):
        status: MotionStatus = self.motion_status.get()
        commanded_positions = jvq_to_list(status.commanded_joint_position)
        names = [f"victor_{self.arm_name}_joint_{i}" for i in range(1, 8)]
        return names, commanded_positions

    def get_jtc_cmd_pub(self, active_controller_name: str):
        if active_controller_name not in self.controller_publishers:
            jtc_cmd_pub = self.node.create_publisher(JointTrajectory, f'{active_controller_name}/joint_trajectory', 10)
            wait_for_subscriber(jtc_cmd_pub)
            self.controller_publishers[active_controller_name] = jtc_cmd_pub

        return self.controller_publishers[active_controller_name]

    def get_joint_cmd_pub(self, active_controller_name: str):
        if active_controller_name not in self.controller_publishers:
            joint_cmd_pub = self.node.create_publisher(Float64MultiArray, f'{active_controller_name}/commands', 10)
            wait_for_subscriber(joint_cmd_pub)
            self.controller_publishers[active_controller_name] = joint_cmd_pub

        return self.controller_publishers[active_controller_name]

    def send_joint_cmd(self, joint_cmd_pub: Publisher, joint_positions):
        """
        Send a Float64MultiArray with joint_positions, ordered joint1 to joint 7.
        """
        if len(joint_positions) != 7:
            raise ValueError(f"joint_positions must be length 7, got {len(joint_positions)}")

        # Check against joint limits
        if np.any(joint_positions < self.lower) or np.any(joint_positions > self.upper):
            raise ValueError(f"Joint positions out of bounds: {joint_positions}")

        active_mode: ControlModeParameters = self.control_mode_listener.get()
        if active_mode.control_mode.mode not in [ControlMode.JOINT_POSITION, ControlMode.JOINT_IMPEDANCE]:
            raise ControlModeError(f"Cannot send joint command in {active_mode} mode")

        msg = Float64MultiArray()
        msg.data = joint_positions

        joint_cmd_pub.publish(msg)

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
        Fills out, validates, and sends a Pose to the cartesian impedance controller.
        """
        active_mode: ControlMode = self.control_mode_listener.get().control_mode
        if active_mode.mode != ControlMode.CARTESIAN_IMPEDANCE:
            raise ControlModeError(f"Cannot send cartesian command in {active_mode} mode")

        if target_hand_in_root.header.frame_id != self.cartesian_cmd_base_frame:
            raise ValueError(f"header.frame_id must be {self.cartesian_cmd_base_frame}")
        if target_hand_in_root.child_frame_id != self.cartesian_cmd_tool_frame:
            raise ValueError(f"child_frame_id must be {self.cartesian_cmd_tool_frame}")

        msg = Pose()
        msg.position.x = target_hand_in_root.transform.translation.x
        msg.position.y = target_hand_in_root.transform.translation.y
        msg.position.z = target_hand_in_root.transform.translation.z
        msg.orientation.w = target_hand_in_root.transform.rotation.w
        msg.orientation.x = target_hand_in_root.transform.rotation.x
        msg.orientation.y = target_hand_in_root.transform.rotation.y
        msg.orientation.z = target_hand_in_root.transform.rotation.z

        self.cartesian_cmd_pub.publish(msg)

    def get_measured_joint_states_from_status(self, include_gripper: bool = True):
        """
        Modified the message in-place

        Args:
            include_gripper: whether to include the gripper in the message
        """
        joint_state = JointState()
        motion_status: MotionStatus = self.get_motion_status()
        joint_state.position.extend(jvq_to_list(motion_status.measured_joint_position))
        joint_state.velocity.extend(jvq_to_list(motion_status.measured_joint_velocity))
        joint_state.effort.extend(jvq_to_list(motion_status.measured_joint_torque))
        joint_state.name.extend([f"victor_{self.arm_name}_joint_{i}" for i in range(1, 8)])

        if not include_gripper:
            return joint_state

        gripper_status: Robotiq3FingerStatus = self.get_gripper_status()

        joint_state.position.extend(compute_finger_angles(gripper_status.finger_a_status.position))
        joint_state.name.extend(get_finger_angle_names(self.name, "finger_a"))

        joint_state.position.append(-compute_scissor_angle(gripper_status.scissor_status.position))
        joint_state.name.append(get_scissor_joint_name(self.name, "finger_b"))

        joint_state.position.extend(compute_finger_angles(gripper_status.finger_b_status.position))
        joint_state.name.extend(get_finger_angle_names(self.name, "finger_b"))

        joint_state.position.append(compute_scissor_angle(gripper_status.scissor_status.position))
        joint_state.name.append(get_scissor_joint_name(self.name, "finger_c"))

        joint_state.position.extend(compute_finger_angles(gripper_status.finger_c_status.position))
        joint_state.name.extend(get_finger_angle_names(self.name, "finger_c"))

        return joint_state

    def get_active_controller_names(self):
        controllers = self.get_active_controllers()
        controller_names = [controller.name for controller in controllers]
        return controller_names

    def get_active_controllers(self) -> List[ControllerState]:
        controllers = self.get_all_controllers()

        # remove inactive controllers
        controllers = [controller for controller in controllers if controller.state == "active"]

        # filter out any control that does not contain any interfaces for this side
        controllers = [controller for controller in controllers if self.is_claimed_by(controller)]
        return controllers

    def get_all_controllers(self) -> List[ControllerState]:
        # Call the list controllers ROS service
        req = ListControllers.Request()
        res: ListControllers.Respotyse = self.list_controllers_client.call(req)
        controllers = res.controller
        # filter out anything with "broadcaster" in the name
        controllers = [controller for controller in controllers if "broadcaster" not in controller.name]
        return controllers

    def is_claimed_by(self, controller: ControllerState):
        return any([self.arm_name in interface for interface in controller.claimed_interfaces])

    def get_control_mode_for_controller(self, controller_name: str) -> str:
        # get the ROS param "control_mode" on the given controller's node
        srv_client = self.node.create_client(GetParameters, f"{controller_name}/get_parameters")
        success = srv_client.wait_for_service(timeout_sec=1.0)
        if not success:
            raise RuntimeError(f"Could not get parameters from {controller_name}")

        req = GetParameters.Request()
        req.names = ["control_mode"]
        res = srv_client.call(req)
        control_mode = res.values[0].string_value

        return control_mode


class Victor:

    def __init__(self, node: Node, robot_description_cb: Optional[Callable[[RobotURDF], None]] = None):
        super().__init__()
        self.node = node
        self.robot_description_user_cb = robot_description_cb

        self.left = Side(node, 'left')
        self.right = Side(node, 'right')

        self.joint_states_listener = Listener(node, JointState, 'joint_states', 10)

        self.cm_srv_group = MutuallyExclusiveCallbackGroup()
        self.switch_controller_client = node.create_client(SwitchController, f"controller_manager/switch_controller",
                                                           callback_group=self.cm_srv_group)
        self.list_controllers_client = node.create_client(ListControllers, f"controller_manager/list_controllers",
                                                          callback_group=self.cm_srv_group)

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
        self.joint_names_urdf_order = [joint.name for joint in self.urdf.joints]
        if self.robot_description_user_cb:
            self.robot_description_user_cb(self.urdf)

    def get_joint_states(self) -> JointState:
        return self.joint_states_listener.get()

    def get_measured_joint_states_from_status(self, include_gripper: bool = True) -> JointState:
        # Make a JointState message from the motion status messages
        joint_state = JointState()

        left_joint_state = self.left.get_measured_joint_states_from_status(include_gripper)
        right_joint_state = self.right.get_measured_joint_states_from_status(include_gripper)

        joint_state.position.extend(left_joint_state.position)
        joint_state.position.extend(right_joint_state.position)
        joint_state.velocity.extend(left_joint_state.velocity)
        joint_state.velocity.extend(right_joint_state.velocity)
        joint_state.effort.extend(left_joint_state.effort)
        joint_state.effort.extend(right_joint_state.effort)
        joint_state.name.extend(left_joint_state.name)
        joint_state.name.extend(right_joint_state.name)

        # sort based on URDF order
        indices = [joint_state.name.index(name) for name in self.joint_names_urdf_order]
        joint_state.name = [joint_state.name[i] for i in indices]
        joint_state.position = [joint_state.position[i] for i in indices]
        joint_state.velocity = [joint_state.velocity[i] for i in indices]
        joint_state.effort = [joint_state.effort[i] for i in indices]

        return joint_state

    def get_motion_statuses(self):
        return {'left': self.left.get_motion_status(), 'right': self.right.get_motion_status()}

    def get_control_modes(self):
        return {'left': self.left.get_arm_control_mode(), 'right': self.right.get_arm_control_mode()}

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
