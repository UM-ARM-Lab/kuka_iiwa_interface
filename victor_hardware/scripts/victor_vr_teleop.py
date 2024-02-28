#!/usr/bin/env python3
"""
This script receives teleop commands from the Unity VR system, and sends them to Victor.
Press and hold the grip button on the VR controller to start recording an episode.
Release the grip button to stop recording an episode.
Press the menu button to stop the script.
Press the trackpad to reset
Use the trigger to open and close the gripper, it's mapped to the open fraction of the gripper.
The motion will be relative in gripper frame to the pose of the gripper when you started recording.
see the vr_ros2_bridge repo for setup instructions.
"""

import time
from pathlib import Path

import numpy as np
import rerun as rr

import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from victor_hardware.victor import Victor
from vr_ros2_bridge_msgs.msg import ControllersInfo, ControllerInfo


def get_identity(parent_frame_id: str, child_frame_id: str):
    msg = TransformStamped()
    msg.transform.rotation.w = 1.
    msg.header.frame_id = parent_frame_id
    msg.child_frame_id = child_frame_id
    return msg


def controller_info_to_se3_pose(controller_info):
    pose_msg: Pose = controller_info.controller_pose
    return TransformStamped(x=pose_msg.position.x,
                            y=pose_msg.position.y,
                            z=pose_msg.position.z,
                            rot=Quat(w=pose_msg.orientation.w,
                                     x=pose_msg.orientation.x,
                                     y=pose_msg.orientation.y,
                                     z=pose_msg.orientation.z))


VISION_FRAME_NAME = "vision"
HAND_FRAME_NAME = "hand"
VR_FRAME_NAME = "vr"
CONTROLLER_FRAME_NAME = "controller"


class GenerateDataVRNode(Node):

    def __init__(self):
        super().__init__("victor_vr_teleop")

        self.victor = Victor(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_action = None
        self.linear_velocity_scale = 1.
        self.hand_in_vision0 = get_identity(VISION_FRAME_NAME, HAND_FRAME_NAME)
        self.controller_in_vr0 = get_identity(VR_FRAME_NAME, CONTROLLER_FRAME_NAME)

        now = int(time.time())
        root = Path(f"data/victor_vr_data_{now}")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.vr_sub = self.create_subscription(ControllersInfo, "vr_controller_info", self.on_controllers_info, 10)

        self.has_started = False
        self.is_recording = False
        self.is_done = False

        self.on_reset()

    def send_cmd(self, target_hand_in_vision: TransformStamped, open_fraction: float):
        hand_pose_msg = target_hand_in_vision.to_proto()
        arm_cmd = RobotCommandBuilder.arm_pose_command_from_pose(hand_pose_msg, VISION_FRAME_NAME, seconds=ARM_POSE_CMD_DURATION)

        self.left_gripper_cmd_pub.publish(get_gripper_closed_fraction_msg(left_open_fraction))

        self.victor.left_arm_cmd_pub.publish(left_arm_cmd)
        self.victor.right_arm_cmd_pub.publish(right_arm_cmd)

    def on_done(self):
        self.victor.open_left_gripper()
        self.victor.open_right_gripper()

        raise SystemExit("Done!")

    def on_controllers_info(self, msg: ControllersInfo):
        if len(msg.controllers_info) == 0:
            return

        controller_info: ControllerInfo = msg.controllers_info[0]

        # set the flags for recording and done
        if not self.is_recording and controller_info.grip_button:
            self.is_recording = True
            self.has_started = True
            self.on_start_recording(controller_info)
        elif self.is_recording and not controller_info.grip_button:
            self.is_recording = False
            self.on_stop_recording()

        if self.has_started and controller_info.menu_button:
            self.is_done = True
            self.on_done()

        if not self.is_recording and controller_info.trackpad_button:
            self.on_reset()

        if self.is_recording:
            # for debugging purposes, we publish a fixed transform from "VR" frame to "VISION" frame,
            # even though we don't ever actually use this transform for controlling the robot.
            vr_to_vision = TransformStamped()
            vr_to_vision.header.stamp = self.get_clock().now().to_msg()
            vr_to_vision.header.frame_id = VR_FRAME_NAME
            vr_to_vision.child_frame_id = VISION_FRAME_NAME
            vr_to_vision.transform.translation.x = 1.5
            vr_to_vision.transform.rotation.w = 1.
            self.tf_broadcaster.sendTransform(vr_to_vision)

            target_hand_in_vision = self.get_target_in_vision(controller_info)

            # hand_in_vision = get_a_tform_b(snapshot, VISION_FRAME_NAME, HAND_FRAME_NAME)
            hand_in_vision: TransformStamped = self.tf

            open_fraction = 1 - controller_info.trigger_axis

            # Save for the data recorder
            self.latest_action = {
                'target_hand_in_vision': target_hand_in_vision,
                'open_fraction': open_fraction
            }

            self.tf_broadcaster.sendTransform(vr_to_vision)
            self.pub_se3_pose_to_tf(self.controller_in_vr0, 'controller_in_vr0', VR_FRAME_NAME)
            self.pub_se3_pose_to_tf(self.hand_in_vision0, 'hand_in_vision0', VISION_FRAME_NAME)
            self.pub_se3_pose_to_tf(hand_in_vision, 'hand_in_vision', VISION_FRAME_NAME)
            self.pub_se3_pose_to_tf(target_hand_in_vision, 'target_in_vision', VISION_FRAME_NAME)

            self.send_cmd(target_hand_in_vision, open_fraction)

    def get_target_in_vision(self, controller_info: ControllerInfo) -> TransformStamped:
        """ Translate delta pose in VR frame to delta pose in VISION frame and send a command to the robot! """
        current_controller_in_vr = controller_info_to_se3_pose(controller_info)
        self.pub_se3_pose_to_tf(current_controller_in_vr, 'current VR', VR_FRAME_NAME)
        delta_in_vr = self.controller_in_vr0.inverse() * current_controller_in_vr

        # TODO: add a rotation here to account for the fact that we might not want VR controller frame and
        #  hand frame to be the same orientation?
        delta_in_vision = delta_in_vr

        target_hand_in_vision = self.hand_in_vision0 * delta_in_vision
        return target_hand_in_vision

    def on_reset(self):
        self.victor.open_left_gripper()
        self.victor.open_right_gripper()

    def on_start_recording(self, controller_info: ControllerInfo):
        # Store the initial pose of the hand in vision frame, as well as the controller pose which is in VR frame
        print(f"Starting recording episode {self.recorder.episode_idx}")

        self.controller_in_vr0 = controller_info_to_se3_pose(controller_info)
        snapshot = self.conq_clients.state.get_robot_state().kinematic_state.transforms_snapshot
        self.hand_in_vision0 = get_a_tform_b(snapshot, VISION_FRAME_NAME, HAND_FRAME_NAME)

    def on_stop_recording(self):
        print(f"Stopping recording episode {self.recorder.episode_idx}")

    def pub_se3_pose_to_tf(self, pose: SE3Pose, child_frame_name: str, parent_frame_name: str):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame_name
        t.child_frame_id = child_frame_name

        t.transform.translation.x = float(pose.x)
        t.transform.translation.y = float(pose.y)
        t.transform.translation.z = float(pose.z)

        t.transform.rotation.x = float(pose.rot.x)
        t.transform.rotation.y = float(pose.rot.y)
        t.transform.rotation.z = float(pose.rot.z)
        t.transform.rotation.w = float(pose.rot.w)

        self.tf_broadcaster.sendTransform(t)

    def get_latest_action(self, _):
        return self.latest_action


def main():
    np.seterr(all='raise')
    np.set_printoptions(precision=3, suppress=True)

    rr.init("generate_data_from_vr")
    rr.connect()

    rclpy.init()

    node = GenerateDataVRNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass

    rclpy.shutdown()
    print("Done!")


if __name__ == '__main__':
    main()
