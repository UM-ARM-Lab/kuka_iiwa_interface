import copy

from rclpy.task import Future
from typing import Sequence, Optional, Callable, List, Union, Tuple, Any
import time
import numpy as np
from transforms3d._gohlketransforms import quaternion_from_euler
import rclpy
from arm_utilities.ros_helpers import wait_for_subscriber
from arm_utilities.listener import Listener
from shape_msgs.msg import SolidPrimitive

from std_msgs.msg import Header, String

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from urdf_parser_py.urdf import Robot as RobotURDF
from urdf_parser_py.urdf import URDF
from urdf_parser_py.xml_reflection import core
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from victor_hardware_interfaces.msg import MotionStatus, Robotiq3FingerStatus, Robotiq3FingerCommand, \
    ControlModeParameters, ControlMode
from victor_python.robotiq_finger_angles import compute_finger_angles, get_finger_angle_names, compute_scissor_angle, \
    get_scissor_joint_name
from victor_python.victor_utils import is_gripper_closed, get_gripper_closed_fraction_msg, jvq_to_list
from arm_robots.robot import load_moveit_config, load_moveitpy
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
        while not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('list controller service not available, waiting again...')
        self.get_get_parameters_client = {}
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
        print("getting active controllers")
        controllers = self.get_all_controllers()

        # remove inactive controllers
        controllers = [controller for controller in controllers if controller.state == "active"]

        # filter out any control that does not contain any interfaces for this side
        controllers = [controller for controller in controllers if self.is_claimed_by(controller)]
        return controllers

    def get_all_controllers(self, mode="async") -> List[ControllerState]:

        # Call the list controllers ROS service
        req = ListControllers.Request()
        if mode == "sync":
            res: ListControllers.Respotyse = self.list_controllers_client.call(req)
        elif mode == "async":
            print("getting the controllers")
            future = self.list_controllers_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            res = future.result()
        print("Got the controllers")
        controllers = res.controller
        controllers = [controller for controller in controllers if "broadcaster" not in controller.name]
        return controllers

    def is_claimed_by(self, controller: ControllerState):
        return any([self.arm_name in interface for interface in controller.claimed_interfaces])

    def get_control_mode_for_controller(self, controller_name: str) -> str:
        print("getting control mode")
        # get the ROS param "control_mode" on the given controller's node
        if controller_name not in self.get_get_parameters_client:
            srv_client = self.node.create_client(GetParameters, f"{controller_name}/get_parameters")
            success = srv_client.wait_for_service(timeout_sec=1.0)
            if not success:
                raise RuntimeError(f"Could not get parameters from {controller_name}")
            self.get_get_parameters_client[controller_name] = srv_client
        else:
            srv_client = self.get_get_parameters_client[controller_name]

        req = GetParameters.Request()
        req.names = ["control_mode"]
        # res = srv_client.call(req)
        future = srv_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        control_mode = res.values[0].string_value

        return control_mode


class Victor:

    def __init__(self, node: Node, robot_description_cb: Optional[Callable[[RobotURDF], None]] = None,
                 callback_group=None,
                 ):
        super().__init__()
        self.node = node
        self.robot_description_user_cb = robot_description_cb

        self.left = Side(node, 'left')
        self.right = Side(node, 'right')
        self.base_link = "victor_root"

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

        callback_group = ReentrantCallbackGroup()
        # Create a service for getting the planning scene
        self._get_planning_scene_service = self.node.create_client(
            srv_type=GetPlanningScene,
            srv_name="get_planning_scene",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )
        self.__planning_scene = None
        self.__old_planning_scene = None
        self.__old_allowed_collision_matrix = None

        # Create a service for applying the planning scene
        self._apply_planning_scene_service = self.node.create_client(
            srv_type=ApplyPlanningScene,
            srv_name="apply_planning_scene",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )
        self.__collision_object_publisher = self.node.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self.urdf: Optional[RobotURDF] = None
        # load moveitpy
        self.moveitpy, self.moveit_config = load_moveitpy("victor")
        self.planning_components = {}

    def set_controller(self, control_mode: str):
        """
          const std::string JOINT_POSITION_INTERFACE = "joint_position";
          const std::string JOINT_IMPEDANCE_INTERFACE = "joint_impedance";
          const std::string CARTESIAN_POSE_INTERFACE = "cartesian_pose";
          const std::string CARTESIAN_IMPEDANCE_INTERFACE = "cartesian_impedance";
          - joint_impedance_trajectory_controller
          - left_arm_position_controller
          - left_arm_impedance_controller
          - left_arm_cartesian_controller
          - right_arm_position_controller
          - right_arm_impedance_controller
          - right_arm_cartesian_controller
          - joint_position_trajectory_controller
          """
        assert control_mode in ["position_controller", "impedance_controller",
                                "joint_position_trajectory_controller", "joint_impedance_trajectory_controller",
                                "cartesian_controller",
                                ]
        left_active_controllers = self.left.get_active_controller_names()
        right_active_controllers = self.right.get_active_controller_names()
        active_controllers = list(set(left_active_controllers + right_active_controllers))
        req = SwitchController.Request()
        req.deactivate_controllers = [controller for controller in active_controllers if controller not in control_mode]
        if control_mode in ["joint_position_trajectory", "joint_impedance_trajectory"]:
            req.activate_controllers = [control_mode]
        else:
            req.activate_controllers = [f"{side}_arm_{control_mode}" for side in ["left", "right"]]

        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        if not res.ok:
            print(f"Failed to switch controllers: {res.ok}")

        return res


    def get_moveit_planning_component(self, group_name):
        if group_name not in self.planning_components:
            self.planning_components[group_name] = self.moveitpy.get_planning_component(group_name)
        return self.planning_components[group_name]

    def plan_to_joint_config(self, group_name: str, joint_config: Union[List, np.ndarray]):
        planning_component = self.get_moveit_planning_component(group_name)
        planning_component.set_start_state_to_current_state()
        psm = self.moveitpy.get_planning_scene_monitor()
        robot_model = self.moveitpy.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.set_joint_group_positions(group_name, joint_config)
        planning_component.set_goal_state(robot_state=robot_state)
        plan_result = planning_component.plan()
        if plan_result:
            robot_trajectory = plan_result.trajectory
            exe_result = self.moveitpy.execute(robot_trajectory, controllers=[])
        else:
            print("Planning failed")
        return plan_result

    def plan_to_pose(self, group_name: str, ee_link_name, target_pose):
        planning_component = self.get_moveit_planning_component(group_name)
        planning_component.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "victor_root"
        pose_goal.pose.position.x = target_pose[0]
        pose_goal.pose.position.y = target_pose[1]
        pose_goal.pose.position.z = target_pose[2]
        if len(target_pose) == 6:
            q = quaternion_from_euler(target_pose[3], target_pose[4], target_pose[5])
            pose_goal.pose.orientation.x = q[0]
            pose_goal.pose.orientation.y = q[1]
            pose_goal.pose.orientation.z = q[2]
            pose_goal.pose.orientation.w = q[3]
        elif len(target_pose) == 7:
            q = np.array(target_pose[3:7])
            q /= np.linalg.norm(q)
            pose_goal.pose.orientation.x = q[0]
            pose_goal.pose.orientation.y = q[1]
            pose_goal.pose.orientation.z = q[2]
            pose_goal.pose.orientation.w = q[3]

        planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=ee_link_name)

        plan_result = planning_component.plan()
        if plan_result:
            robot_trajectory = plan_result.trajectory
            exe_result = self.moveitpy.execute(robot_trajectory, controllers=[])
        else:
            print("Planning failed")
        return plan_result

    @property
    def planning_scene(self) -> Optional[PlanningScene]:
        return self.__planning_scene

    def add_collision_primitive(
            self,
            id: str,
            primitive_type: int,
            dimensions: Tuple[float, float, float],
            pose: Optional[Union[PoseStamped, Pose]] = None,
            position: Optional[Union[Point, Tuple[float, float, float]]] = None,
            quat_xyzw: Optional[
                Union[Quaternion, Tuple[float, float, float, float]]
            ] = None,
            frame_id: Optional[str] = None,
            operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a primitive geometry specified by its dimensions.

        `primitive_type` can be one of the following:
            - `SolidPrimitive.BOX`
            - `SolidPrimitive.SPHERE`
            - `SolidPrimitive.CYLINDER`
            - `SolidPrimitive.CONE`
        """

        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.base_link
                    ),
                ),
                pose=pose,
            )
        else:
            if not isinstance(position, Point):
                position = Point(
                    x=float(position[0]), y=float(position[1]), z=float(position[2])
                )
            if not isinstance(quat_xyzw, Quaternion):
                quat_xyzw = Quaternion(
                    x=float(quat_xyzw[0]),
                    y=float(quat_xyzw[1]),
                    z=float(quat_xyzw[2]),
                    w=float(quat_xyzw[3]),
                )
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.base_link
                    ),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        msg.primitives.append(
            SolidPrimitive(type=primitive_type, dimensions=dimensions)
        )

        self.__collision_object_publisher.publish(msg)

    def add_collision_box(
            self,
            id: str,
            size: Tuple[float, float, float],
            pose: Optional[Union[PoseStamped, Pose]] = None,
            position: Optional[Union[Point, Tuple[float, float, float]]] = None,
            quat_xyzw: Optional[
                Union[Quaternion, Tuple[float, float, float, float]]
            ] = None,
            frame_id: Optional[str] = None,
            operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a box geometry specified by its size.
        """

        assert len(size) == 3, "Invalid size of the box!"

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.BOX,
            dimensions=size,
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_sphere(
            self,
            id: str,
            radius: float,
            pose: Optional[Union[PoseStamped, Pose]] = None,
            position: Optional[Union[Point, Tuple[float, float, float]]] = None,
            quat_xyzw: Optional[
                Union[Quaternion, Tuple[float, float, float, float]]
            ] = None,
            frame_id: Optional[str] = None,
            operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a sphere geometry specified by its radius.
        """

        if quat_xyzw is None:
            quat_xyzw = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.SPHERE,
            dimensions=[
                radius,
            ],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_cylinder(
            self,
            id: str,
            height: float,
            radius: float,
            pose: Optional[Union[PoseStamped, Pose]] = None,
            position: Optional[Union[Point, Tuple[float, float, float]]] = None,
            quat_xyzw: Optional[
                Union[Quaternion, Tuple[float, float, float, float]]
            ] = None,
            frame_id: Optional[str] = None,
            operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a cylinder geometry specified by its height and radius.
        """

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.CYLINDER,
            dimensions=[height, radius],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_cone(
            self,
            id: str,
            height: float,
            radius: float,
            pose: Optional[Union[PoseStamped, Pose]] = None,
            position: Optional[Union[Point, Tuple[float, float, float]]] = None,
            quat_xyzw: Optional[
                Union[Quaternion, Tuple[float, float, float, float]]
            ] = None,
            frame_id: Optional[str] = None,
            operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a cone geometry specified by its height and radius.
        """

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.CONE,
            dimensions=[height, radius],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_mesh(
            self,
            filepath: Optional[str],
            id: str,
            pose: Optional[Union[PoseStamped, Pose]] = None,
            position: Optional[Union[Point, Tuple[float, float, float]]] = None,
            quat_xyzw: Optional[
                Union[Quaternion, Tuple[float, float, float, float]]
            ] = None,
            frame_id: Optional[str] = None,
            operation: int = CollisionObject.ADD,
            scale: Union[float, Tuple[float, float, float]] = 1.0,
            mesh: Optional[Any] = None,
    ):
        """
        Add collision object with a mesh geometry. Either `filepath` must be
        specified or `mesh` must be provided.
        Note: This function required 'trimesh' Python module to be installed.
        """

        # Load the mesh
        try:
            import trimesh
        except ImportError as err:
            raise ImportError(
                "Python module 'trimesh' not found! Please install it manually in order "
                "to add collision objects into the MoveIt 2 planning scene."
            ) from err

        # Check the parameters
        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )
        if (filepath is None and mesh is None) or (
                filepath is not None and mesh is not None
        ):
            raise ValueError("Exactly one of `filepath` or `mesh` must be specified!")
        if mesh is not None and not isinstance(mesh, trimesh.Trimesh):
            raise ValueError("`mesh` must be an instance of `trimesh.Trimesh`!")

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.base_link
                    ),
                ),
                pose=pose,
            )
        else:
            if not isinstance(position, Point):
                position = Point(
                    x=float(position[0]), y=float(position[1]), z=float(position[2])
                )
            if not isinstance(quat_xyzw, Quaternion):
                quat_xyzw = Quaternion(
                    x=float(quat_xyzw[0]),
                    y=float(quat_xyzw[1]),
                    z=float(quat_xyzw[2]),
                    w=float(quat_xyzw[3]),
                )
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.base_link
                    ),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        if filepath is not None:
            mesh = trimesh.load(filepath)

        # Scale the mesh
        if isinstance(scale, float):
            scale = (scale, scale, scale)
        if not (scale[0] == scale[1] == scale[2] == 1.0):
            # If the mesh was passed in as a parameter, make a copy of it to
            # avoid transforming the original.
            if filepath is not None:
                mesh = mesh.copy()
            # Transform the mesh
            transform = np.eye(4)
            np.fill_diagonal(transform, scale)
            mesh.apply_transform(transform)

        msg.meshes.append(
            Mesh(
                triangles=[MeshTriangle(vertex_indices=face) for face in mesh.faces],
                vertices=[
                    Point(x=vert[0], y=vert[1], z=vert[2]) for vert in mesh.vertices
                ],
            )
        )

        self.__collision_object_publisher.publish(msg)

    def remove_collision_object(self, id: str):
        """
        Remove collision object specified by its `id`.
        """

        msg = CollisionObject()
        msg.id = id
        msg.operation = CollisionObject.REMOVE
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.__collision_object_publisher.publish(msg)

    def remove_collision_mesh(self, id: str):
        """
        Remove collision mesh specified by its `id`.
        Identical to `remove_collision_object()`.
        """

        self.remove_collision_object(id)

    def update_planning_scene(self) -> bool:
        """
        Gets the current planning scene. Returns whether the service call was
        successful.
        """

        if not self._get_planning_scene_service.service_is_ready():
            self.node.get_logger().warn(
                f"Service '{self._get_planning_scene_service.srv_name}' is not yet available. Better luck next time!"
            )
            return False
        self.__planning_scene = self._get_planning_scene_service.call(
            GetPlanningScene.Request()
        ).scene
        return True

    def clear_all_collision_objects(self) -> Optional[Future]:
        """
        Removes all attached and un-attached collision objects from the planning scene.

        Returns a future for the ApplyPlanningScene service call.
        """
        # Update the planning scene
        if not self.update_planning_scene():
            return None
        self.__old_planning_scene = copy.deepcopy(self.__planning_scene)

        # Remove all collision objects from the planning scene
        self.__planning_scene.world.collision_objects = []
        self.__planning_scene.robot_state.attached_collision_objects = []

        # Apply the new planning scene
        if not self._apply_planning_scene_service.service_is_ready():
            self.node.get_logger().warn(
                f"Service '{self._apply_planning_scene_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None
        return self._apply_planning_scene_service.call_async(
            ApplyPlanningScene.Request(scene=self.__planning_scene)
        )

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
