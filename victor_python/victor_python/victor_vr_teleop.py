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
from time import perf_counter

import numpy as np
import transforms3d
from moveit.core.robot_model import RobotModel, JointModelGroup
from moveit.core.robot_state import RobotState
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from victor_hardware_interfaces.msg import MotionStatus
from vr_ros2_bridge_msgs.msg import ControllersInfo, ControllerInfo

import rclpy
from arm_utilities.numpy_conversions import transform_to_mat, mat_to_transform
from arm_utilities.transformation_helper import np_tf_inv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from victor_python.victor import Victor, Side
from victor_python.victor_moveitpy import VictorMoveItPy
from victor_python.victor_utils import get_gripper_closed_fraction_msg, jvq_to_list

VR_FRAME_NAME = "vr"


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

    def __init__(self, node: Node, side: Side, victorpy: VictorMoveItPy, tf_broadcaster: TransformBroadcaster,
                 tf_buffer: Buffer):
        self.node = node
        self.side = side
        self.victorpy = victorpy
        self.tf_broadcaster = tf_broadcaster
        self.tf_buffer = tf_buffer

        self.controller_in_vr0 = np.eye(4)
        self.tool_in_base0 = np.eye(4)

        self.robot_model: RobotModel = self.victorpy.robot_model
        self.jmg: JointModelGroup = self.robot_model.get_joint_model_group(self.side.arm_name)
        self.planning_component = self.victorpy.moveitpy.get_planning_component(self.side.arm_name)
        self.planning_scene_monitor = self.victorpy.moveitpy.get_planning_scene_monitor()
        # FIXME: Python API not working to get this info?
        self.tool_frame = 'victor_right_tool0'
        self.base_frame = 'victor_root'
        # tool_frame = jmg.eef_name  # not working???
        # base_frame = robot_model.get_model_info()

    def on_start_recording(self, controller_info: ControllerInfo):
        controller_in_vr0_msg = controller_info_to_tf(self.node, controller_info)
        self.controller_in_vr0 = transform_to_mat(controller_in_vr0_msg.transform)

        self.tf_broadcaster.sendTransform(controller_in_vr0_msg)

        tool_in_base0_msg = self.tf_buffer.lookup_transform(self.base_frame, self.tool_frame, rclpy.time.Time())
        self.tool_in_base0 = transform_to_mat(tool_in_base0_msg.transform)

    def on_stop_recording(self):
        pass

    def send_cmd(self, controller_info: ControllerInfo, open_fraction: float):
        joint_positions = self.get_target_in_base(controller_info)
        if joint_positions is not None:
            self.side.send_joint_cmd(joint_positions)
        self.side.gripper_command.publish(get_gripper_closed_fraction_msg(open_fraction))

    def get_target_in_base(self, controller_info: ControllerInfo) -> TransformStamped:
        current_controller_in_vr_msg = controller_info_to_tf(self.node, controller_info)
        current_controller_in_vr = transform_to_mat(current_controller_in_vr_msg.transform)
        delta_in_controller = np_tf_inv(self.controller_in_vr0) @ current_controller_in_vr

        # rotate delta_in_controller by 180 about Z to make it so the operator can face the robot
        rotate_33 = transforms3d.euler.euler2mat(0, 0, np.pi)

        current_state = RobotState(self.robot_model)
        motion_status: MotionStatus = self.side.motion_status.get()
        current_cmd_positions = jvq_to_list(motion_status.commanded_joint_position)
        current_state.set_joint_group_positions(self.side.arm_name, current_cmd_positions)
        current_state.update()

        tool_in_base = current_state.get_global_link_transform(self.tool_frame)

        tool_to_base_rot_mat = tool_in_base[:3, :3]
        target_in_base_rot_mat = delta_in_controller[:3, :3] @ tool_to_base_rot_mat
        target_in_base = np.eye(4)
        target_in_base[:3, :3] = target_in_base_rot_mat
        target_in_base[:3, 3] = tool_in_base[:3, 3] + rotate_33 @ delta_in_controller[:3, 3]

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
                # print('before:', robot_state.get_joint_group_positions(self.side.arm_name))
                # print('       ', robot_state.get_pose(self.tool_frame))
                break
            else:
                break

        ik_t1 = perf_counter()
        print(f"IK took {ik_t1 - ik_t0:.3f} seconds")

        if success:
            joint_positions = robot_state.get_joint_group_positions(self.side.arm_name)
            return joint_positions
        else:
            print("IK failed!")
            return None


class VictorTeleopNode(Node):

    def __init__(self):
        super().__init__("victor_vr_teleop")

        self.victor = Victor(self)
        self.victorpy = VictorMoveItPy(self.victor)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_action_dict = None
        self.linear_velocity_scale = 1.

        self.tf_broadcaster = TransformBroadcaster(self)
        self.vr_sub = self.create_subscription(ControllersInfo, "vr_controller_info", self.on_controllers_info, 10)

        self.left = SideTeleop(self, self.victor.left, self.victorpy, self.tf_broadcaster, self.tf_buffer)
        self.right = SideTeleop(self, self.victor.right, self.victorpy, self.tf_broadcaster, self.tf_buffer)

        self.has_started = False
        self.is_recording = False
        self.is_done = False

        self.rcv_dts = []
        self.last_rcv_t = perf_counter()

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
                if controller_info.grip_button:
                    open_fraction = controller_info.trigger_axis
                    if 'left' in controller_info.controller_name:
                        self.latest_action_dict['left_open_fraction'] = open_fraction
                        # self.latest_action_dict['left_target_in_base'] = left_target_in_base
                        self.left.send_cmd(controller_info, open_fraction)
                    elif 'right' in controller_info.controller_name:
                        self.latest_action_dict['right_open_fraction'] = open_fraction
                        # self.latest_action_dict['right_target_in_base'] = right_target_in_base
                        self.right.send_cmd(controller_info, open_fraction)

        now = perf_counter()
        rcv_dt = self.last_rcv_t - now
        self.rcv_dts.append(rcv_dt)
        if len(self.rcv_dts) > 100:
            self.rcv_dts.pop(0)
        mean_rcv_dt = np.mean(self.rcv_dts)
        if mean_rcv_dt > 0.038:
            print(f'{mean_rcv_dt=:.3f}')

        self.last_rcv_t = now

    def on_done(self):
        self.victor.left.open_gripper()
        self.victor.right.open_gripper()

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
