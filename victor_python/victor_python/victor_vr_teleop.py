#!/usr/bin/env python3
"""
This script receives teleop commands from the Unity VR system, and sends them to Victor.
Press and hold the grip button on the VR controller to start recording an episode.
Release the grip button to stop recording an episode.
Press the menu button to stop the script.
Use the trigger to open and close the gripper, it's mapped to the open fraction of the gripper.
The motion will be relative in gripper frame to the pose of the gripper when you started recording.
see the vr_ros2_bridge repo for setup instructions.
"""
from copy import deepcopy
from threading import Thread
from time import perf_counter
from typing import Callable

import numpy as np
import transforms3d

import rclpy
from ament_index_python import get_package_share_path
from arm_utilities.filters import BatchOnlineFilter
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
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from victor_hardware_interfaces.msg import MotionStatus, ControlMode, Robotiq3FingerStatus
from victor_hardware_interfaces.srv import SetControlMode
from victor_python.victor import Victor, Side
from victor_python.victor_utils import get_control_mode_params
from victor_python.victor_utils import get_gripper_closed_fraction_msg, jvq_to_list
from vr_ros2_bridge_msgs.msg import ControllersInfo, ControllerInfo

VR_FRAME_NAME = "vr"

class ButtonClicker:

    def __init__(self, callback: Callable):
        self.callback = callback
        self.has_triggered = False

    def update(self, is_pressed: bool):
        if not self.has_triggered and is_pressed:
            self.has_triggered = True
            self.callback()
        elif self.has_triggered and not is_pressed:
            self.has_triggered = False


class AxisVelocityTracker:

    def __init__(self, name, dt, deadzone, min=0., max=1.):
        self.last_position = None
        self.name = name
        self.position = None
        self.dt = dt
        self.deadzone = deadzone
        self.min = min
        self.max = max

    def update(self, current_position, velocity):
        if abs(velocity) < self.deadzone:
            velocity = 0.0
        self.position = current_position + velocity * self.dt
        self.position = np.clip(self.position, self.min, self.max)

        # print(self.name, current_position, velocity, self.position, self.last_position)
        if self.last_position:
            needs_to_send = abs(self.position - self.last_position) >= self.deadzone * self.dt
        else:
            needs_to_send = False

        self.last_position = self.position

        return needs_to_send

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


class SideTeleop:

    def __init__(self, node: Node, side: Side, moveitpy: MoveItPy, tf_broadcaster: TransformBroadcaster,
                 tf_buffer: Buffer):
        self.gripper_in_motion = None
        self.node = node
        self.side = side
        self.moveitpy = moveitpy
        self.robot_model: RobotModel = self.moveitpy.get_robot_model()
        self.tf_broadcaster = tf_broadcaster
        self.tf_buffer = tf_buffer
        self.gripper_open = True

        self.close_axis_velocity_tracker = AxisVelocityTracker(self.side.name + "close", dt=0.01, deadzone=0.5)
        self.scissor_axis_velocity_tracker = AxisVelocityTracker(self.side.name + "scissor", dt=0.01, deadzone=0.5)

        self.controller_in_vr0 = np.eye(4)
        self.tool_in_base0 = np.eye(4)

        self.base_frame = self.robot_model.model_frame
        self.jmg: JointModelGroup = self.robot_model.get_joint_model_group(self.side.arm_name)
        self.tool_frame = self.jmg.eef_name

        self.filter_pub = self.node.create_publisher(JointState, 'filtered_joint_states', 10)

        self.filter = BatchOnlineFilter(numtaps=20, cutoff=0.1)


        self.trackpad_clicker = ButtonClicker(callback=self.toggle_gripper)

    def on_start_recording(self, controller_info: ControllerInfo):
        controller_in_vr0_msg = controller_info_to_tf(self.node, controller_info)
        self.controller_in_vr0 = transform_to_mat(controller_in_vr0_msg.transform)

        self.tf_broadcaster.sendTransform(controller_in_vr0_msg)

        current_state, self.tool_in_base0 = self.get_current_commanded_tool()

        tool_in_base0_msg = TransformStamped()
        tool_in_base0_msg.transform = mat_to_transform(self.tool_in_base0)
        tool_in_base0_msg.header.frame_id = self.base_frame
        tool_in_base0_msg.child_frame_id = f"tool_{self.side.arm_name}_in_base0"
        self.tf_broadcaster.sendTransform(tool_in_base0_msg)

        controller_in_vr0_msg.child_frame_id = f"controller_{self.side.arm_name}_in_vr0"
        self.tf_broadcaster.sendTransform(controller_in_vr0_msg)

    def on_stop_recording(self):
        pass

    def send_cmd(self, controller_info: ControllerInfo):
        joint_positions = self.get_target_in_base(controller_info)

        if joint_positions is not None:
            # TODO: use filtered joint positions!
            self.side.send_joint_cmd(joint_positions)

            joint_positions_filtered = self.filter.update(joint_positions)
            joint_state_msg = JointState()
            joint_state_msg.name = [f"victor_{self.side.arm_name}_joint_{i}" for i in
                                    range(len(joint_positions_filtered))]
            joint_state_msg.position = joint_positions_filtered.squeeze().tolist()
            self.filter_pub.publish(joint_state_msg)

        # use Y axis as gripper open velocity
        gripper_status: Robotiq3FingerStatus = self.side.gripper_status.get()
        current_closed_fraction = gripper_status.finger_a_status.position
        current_scissor_position = gripper_status.scissor_status.position

        close_needs_send = self.close_axis_velocity_tracker.update(current_closed_fraction, controller_info.trackpad_axis_touch_y)
        scissor_needs_send = self.scissor_axis_velocity_tracker.update(current_scissor_position, -controller_info.trackpad_axis_touch_x)


        if close_needs_send or scissor_needs_send:
            self.side.gripper_command.publish(get_gripper_closed_fraction_msg(self.close_axis_velocity_tracker.position,
                                                                              self.scissor_axis_velocity_tracker.position))

        return {
            'right_open_fraction': self.close_axis_velocity_tracker.position,
            'scissor_position': self.scissor_axis_velocity_tracker.position,
            'joint_positions': joint_positions,
        }

    def toggle_gripper(self):
        self.gripper_open = not self.gripper_open

        gripper_status: Robotiq3FingerStatus = self.side.gripper_status.get()
        current_scissor_position = gripper_status.scissor_status.position

        if self.gripper_open:
            self.side.open_gripper(current_scissor_position)
        else:
            self.side.close_gripper(current_scissor_position)

    def get_target_in_base(self, controller_info: ControllerInfo) -> TransformStamped:
        current_controller_in_vr_msg = controller_info_to_tf(self.node, controller_info)
        current_controller_in_vr = transform_to_mat(current_controller_in_vr_msg.transform)
        delta_in_controller = np_tf_inv(self.controller_in_vr0) @ current_controller_in_vr

        current_state, tool_in_base = self.get_current_commanded_tool()

        target_in_base = self.tool_in_base0 @ delta_in_controller

        self.tf_broadcaster.sendTransform(current_controller_in_vr_msg)
        target_tool_in_base_msg = TransformStamped()
        target_tool_in_base_msg.transform = mat_to_transform(target_in_base)
        target_tool_in_base_msg.header.frame_id = self.base_frame

        # Just for debugging!
        target_tool_in_base_msg.child_frame_id = f"target_{self.tool_frame}"
        self.tf_broadcaster.sendTransform(target_tool_in_base_msg)

        # Solve IK
        pose_goal = Pose()
        pose_goal.position.x = target_tool_in_base_msg.transform.translation.x
        pose_goal.position.y = target_tool_in_base_msg.transform.translation.y
        pose_goal.position.z = target_tool_in_base_msg.transform.translation.z
        pose_goal.orientation = target_tool_in_base_msg.transform.rotation

        ik_t0 = perf_counter()
        success = False
        robot_state = deepcopy(current_state)
        while True:
            ok = robot_state.set_from_ik(self.side.arm_name, pose_goal, self.tool_frame)
            if ok:
                success = True
                break
            else:
                break

        ik_t1 = perf_counter()
        # print(f"IK took {ik_t1 - ik_t0:.3f} seconds")

        if success:
            joint_positions = robot_state.get_joint_group_positions(self.side.arm_name)
            return joint_positions
        else:
            print("IK failed!")
            return None

    def get_current_commanded_tool(self):
        current_state = RobotState(self.robot_model)
        motion_status: MotionStatus = self.side.motion_status.get()
        current_cmd_positions = jvq_to_list(motion_status.commanded_joint_position)
        current_state.set_joint_group_positions(self.side.arm_name, current_cmd_positions)
        current_state.update()
        tool_in_base = current_state.get_global_link_transform(self.tool_frame)
        return current_state, tool_in_base

    def update(self, controller_info: ControllerInfo):
        gripper_status: Robotiq3FingerStatus = self.side.gripper_status.get()
        self.gripper_in_motion = gripper_status.gripper_motion_status == Robotiq3FingerStatus.GRIPPER_IN_MOTION

        self.trackpad_clicker.update(controller_info.trackpad_button)


class VictorTeleopNode(Node):

    def __init__(self):
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_action_dict = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.vr_sub = self.create_subscription(ControllersInfo, "vr_controller_info", self.on_controllers_info, 10)

        self.left = SideTeleop(self, self.victor.left, self.moveitpy, self.tf_broadcaster, self.tf_buffer)
        self.right = SideTeleop(self, self.victor.right, self.moveitpy, self.tf_broadcaster, self.tf_buffer)

        self.has_started = False
        self.is_recording = False
        self.is_done = False

        self.rcv_dts = []
        self.last_rcv_t = perf_counter()

        self.set_control_modes_async()

    def set_control_modes_async(self):
        thread = Thread(target=self.set_control_modes)
        thread.start()

    def set_control_modes(self):
        req = SetControlMode.Request()
        req.new_control_mode = get_control_mode_params(ControlMode.JOINT_IMPEDANCE, vel=1.0, accel=0.1)

        self.victor.left.set_control_mode_client.call(req)
        self.victor.right.set_control_mode_client.call(req)

    def on_controllers_info(self, msg: ControllersInfo):
        if len(msg.controllers_info) == 0:
            return

        any_grip_button = any([controller_info.grip_button for controller_info in msg.controllers_info])
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

        for controller_info in msg.controllers_info:
            if 'left' in controller_info.controller_name:
                self.left.update(controller_info)
            elif 'right' in controller_info.controller_name:
                self.right.update(controller_info)

        # set the flags for recording and done
        if not self.is_recording and any_grip_button:
            self.is_recording = True
            self.has_started = True
            for controller_info in msg.controllers_info:
                if 'left' in controller_info.controller_name:
                    self.left.on_start_recording(controller_info)
                elif 'right' in controller_info.controller_name:
                    self.right.on_start_recording(controller_info)
        elif self.is_recording and not any_grip_button:
            self.is_recording = False
            for controller_info in msg.controllers_info:
                if 'left' in controller_info.controller_name:
                    self.left.on_stop_recording()
                elif 'right' in controller_info.controller_name:
                    self.right.on_stop_recording()

        if self.has_started and any_menu_button:
            self.is_done = True
            self.on_done()

        if self.is_recording:
            self.latest_action_dict = {}
            controller_info: ControllerInfo
            for controller_info in msg.controllers_info:
                if 'left' in controller_info.controller_name:
                    if controller_info.grip_button:
                        left_action = self.left.send_cmd(controller_info)
                        self.latest_action_dict.update(left_action)
                    if controller_info.trackpad_button:
                        self.left.toggle_gripper()
                elif 'right' in controller_info.controller_name:
                    if controller_info.grip_button:
                        right_action = self.right.send_cmd(controller_info)
                        self.latest_action_dict.update(right_action)
                    if controller_info.trackpad_button:
                        self.right.toggle_gripper()



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
        raise SystemExit("Done!")


def main():
    np.seterr(all='raise')
    np.set_printoptions(precision=3, suppress=True)

    rclpy.init()

    node = VictorTeleopNode()

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
