#!/usr/bin/env python3
import signal
import sys
import threading
from copy import deepcopy
from pathlib import Path

import numpy as np
from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout
from victor_hardware_interfaces.msg import Robotiq3FingerCommand, Robotiq3FingerStatus, \
    Robotiq3FingerActuatorStatus, Robotiq3FingerActuatorCommand

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import Robot
from victor_python.victor import Victor, Side, ROBOTIQ_OPEN, ROBOTIQ_CLOSED
from victor_python.victor_utils import get_gripper_closed_fraction_msg

ui_dir = Path(__file__).resolve().parent


def joint_angle_to_slider_pos(pos):
    return int(np.rad2deg(pos))


def slider_pos_to_joint_angle(slider_pos):
    return np.deg2rad(slider_pos)


class VictorCommandWindow(QMainWindow):
    def __init__(self, node: Node, parent=None):
        QMainWindow.__init__(self, parent)
        uic.loadUi(ui_dir / 'victor_command_gui.ui', self)
        self.node = node

        self.victor = Victor(node, robot_description_cb=self.robot_description_callback)

        # self.thread_pool = ThreadPool

        self.left_arm_widget = ArmWidget(node, self.victor, 'left_arm', self.victor.left)
        self.right_arm_widget = ArmWidget(node, self.victor, 'right_arm', self.victor.right)

        self.left_motion_group.layout().addWidget(self.left_arm_widget)
        self.right_motion_group.layout().addWidget(self.right_arm_widget)

    def robot_description_callback(self, robot: Robot):
        # NOTE: this callback gets called on its own thread, so don't directly call functions on QtWidgets here.
        self.left_arm_widget.on_robot_description(robot)
        self.right_arm_widget.on_robot_description(robot)

        self.left_arm_widget.after_initial_robot_data.emit()
        self.right_arm_widget.after_initial_robot_data.emit()


class ArmWidget(QWidget):
    initial_robot_data = pyqtSignal(Robot, Robotiq3FingerStatus)
    after_initial_robot_data = pyqtSignal()
    activeControllerChanged = pyqtSignal(str, str, int)

    def __init__(self, node: Node, victor: Victor, arm_name: str, side: Side):
        QWidget.__init__(self)
        uic.loadUi(ui_dir / 'arm_widget.ui', self)
        self.node = node
        self.victor = victor
        self.arm_name = arm_name
        self.side = side

        # Create the command objects which we will update in the callbacks and publish
        self.gripper_cmd = Robotiq3FingerCommand()

        # Add slider widgets for each joint, but we don't have values yet
        self.slider_widgets = []
        j_lower, j_upper = np.rad2deg(side.lower).astype(np.int), np.rad2deg(side.upper).astype(np.int)
        for joint_idx in range(7):
            slider_widget = ArmJointSliderWidget(f"Joint {joint_idx + 1} Command", joint_idx)
            slider_widget.slider.setRange(j_lower[joint_idx], j_upper[joint_idx])
            slider_widget.setEnabled(False)
            self.slider_widgets.append(slider_widget)
            self.arm_layout.addWidget(slider_widget)

        # Add widgets for each finger
        self.finger_a_widget = FingerWidget("Finger A")
        self.finger_b_widget = FingerWidget("Finger B")
        self.finger_c_widget = FingerWidget("Finger C")
        self.scissor_widget = FingerWidget("Scissor")
        self.fingers_layout.addWidget(self.finger_a_widget)
        self.fingers_layout.addWidget(self.finger_b_widget)
        self.fingers_layout.addWidget(self.finger_c_widget)
        self.fingers_layout.addWidget(self.scissor_widget)

        # connect
        self.open_gripper_button.clicked.connect(self.on_open_gripper)
        self.close_gripper_button.clicked.connect(self.on_close_gripper)
        self.reset_sliders_button.clicked.connect(self.reset_sliders)
        self.finger_a_widget.cmdChanged.connect(self.on_finger_a_cmd_changed)
        self.finger_b_widget.cmdChanged.connect(self.on_finger_b_cmd_changed)
        self.finger_c_widget.cmdChanged.connect(self.on_finger_c_cmd_changed)
        self.scissor_widget.cmdChanged.connect(self.on_scissor_cmd_changed)

        self.activeControllerChanged.connect(self.setControllerText)
        self.initial_robot_data.connect(self.on_initial_robot_data)
        self.after_initial_robot_data.connect(self.connect_and_enable_sliders)

        # Start things as disabled, since we have to wait for the robot description to enable them
        self.reset_sliders_button.setEnabled(False)

        # Periodically check for changes in which ROS2 controllers are running
        self.timer = QTimer()
        self.timer.timeout.connect(self.periodic_update_async)
        self.timer.start(1000)

    def setControllerText(self, controller_name: str, expected_control_mode: str, active_control_mode: int):
        self.active_controller_edit.setText(controller_name)
        self.expected_control_mode_edit.setText(expected_control_mode)
        self.active_control_mode_edit.setText(str(active_control_mode))

    def periodic_update_async(self):
        thread = threading.Thread(target=self.periodic_update)
        thread.start()

    def periodic_update(self):
        active_control_mode = self.side.control_mode_listener.get().control_mode.mode

        active_controller_names = self.side.get_active_controller_names()

        if len(active_controller_names) == 0:
            self.activeControllerChanged.emit("No active controllers", "", active_control_mode)
        elif len(active_controller_names) > 1:
            self.activeControllerChanged.emit("Multiple active controllers!!! Probably a bug...", "",
                                              active_control_mode)
        else:
            active_controller_name = active_controller_names[0]
            control_mode = self.side.get_control_mode_for_controller(active_controller_name)

            self.activeControllerChanged.emit(active_controller_name, control_mode, active_control_mode)

    def on_robot_description(self, robot_description: Robot):
        """ Called off the main thread """
        print(f"waiting for {self.side.name} gripper status...")
        gripper_status: Robotiq3FingerStatus = self.side.gripper_status.get()
        print(f"got {self.side.name} gripper status")

        # Emit back to the main thread
        self.initial_robot_data.emit(robot_description, gripper_status)

    def on_initial_robot_data(self,
                              robot: Robot,
                              gripper_status: Robotiq3FingerStatus):
        """ Called on the main thread """

        # Set the values of the gripper sliders
        self.finger_a_widget.set_status(gripper_status.finger_a_status)
        self.finger_b_widget.set_status(gripper_status.finger_b_status)
        self.finger_c_widget.set_status(gripper_status.finger_c_status)
        self.scissor_widget.set_status(gripper_status.scissor_status)

        # Enable things
        self.reset_sliders_button.setEnabled(True)

        self.reset_sliders()

        # Connect the sliders
        for joint_idx in range(7):
            expected_joint_name = f"victor_{self.arm_name}_joint_{joint_idx + 1}"
            joint = robot.joint_map[expected_joint_name]
            lower_deg = joint_angle_to_slider_pos(joint.limit.lower)
            upper_deg = joint_angle_to_slider_pos(joint.limit.upper)

            slider_widget = self.slider_widgets[joint_idx]
            slider_widget.slider.setRange(lower_deg, upper_deg)

    def connect_and_enable_sliders(self):
        for joint_idx in range(7):
            slider_widget = self.slider_widgets[joint_idx]
            slider_widget.setEnabled(True)
            slider_widget.connect()
            slider_widget.jointChanged.connect(self.publish_arm_cmd_async)

    def reset_sliders(self):
        # Read the current joint states and update the sliders
        joint_states_dict = self.victor.get_joint_cmd_dict()
        print("Resetting slider ", self.arm_name)
        print(joint_states_dict)
        for joint_idx in range(7):
            expected_joint_name = f"victor_{self.arm_name}_joint_{joint_idx + 1}"
            current_rad = joint_states_dict[expected_joint_name]
            slider_pos = joint_angle_to_slider_pos(current_rad)
            print(slider_pos)
            self.slider_widgets[joint_idx].set_value(slider_pos)

    def on_finger_a_cmd_changed(self, cmd: Robotiq3FingerActuatorCommand):
        self.gripper_cmd.finger_a_command = cmd
        self.publish_gripper_cmd()

    def on_finger_b_cmd_changed(self, cmd: Robotiq3FingerActuatorCommand):
        self.gripper_cmd.finger_b_command = cmd
        self.publish_gripper_cmd()

    def on_finger_c_cmd_changed(self, cmd: Robotiq3FingerActuatorCommand):
        self.gripper_cmd.finger_c_command = cmd
        self.publish_gripper_cmd()

    def on_scissor_cmd_changed(self, cmd: Robotiq3FingerActuatorCommand):
        self.gripper_cmd.scissor_command = cmd
        self.publish_gripper_cmd()

    def publish_arm_cmd_async(self, slider_idx: int, joint_angle: float):
        thread = threading.Thread(target=self.publish_arm_cmd, args=(slider_idx, joint_angle))
        thread.start()

    def publish_arm_cmd(self, slider_idx: int, joint_angle: float):
        # First we need to infer which controller is running, and then send the command to that controller
        active_controllers = self.side.get_active_controllers()
        if len(active_controllers) == 0:
            print("No active controllers")
            return
        elif len(active_controllers) > 1:
            print("Multiple active controllers!!! Probably a bug...")
            return

        active_controller = active_controllers[0]
        current_commanded_positions_dict = self.victor.get_joint_cmd_dict()
        print(active_controller.type, active_controller.name)

        # victor_hardware/KukaJointTrajectoryController joint_impedance_trajectory_controller
        if active_controller.type == 'victor_hardware/KukaJointTrajectoryController':
            joint_names_for_controller = []
            current_commanded_positions_for_controller = []

            for cmd_if in active_controller.required_command_interfaces:
                if 'position' in cmd_if:
                    joint_name = cmd_if.split('/')[0]
                    joint_names_for_controller.append(joint_name)
                    current_commanded_positions_for_controller.append(current_commanded_positions_dict[joint_name])
            print(current_commanded_positions_for_controller)

            # Initialize the end positions to the current commanded positions,
            # then update the ones for the sliders of this side
            end_positions = deepcopy(current_commanded_positions_for_controller)

            joint_name = f"victor_{self.arm_name}_joint_{slider_idx + 1}"
            joint_idx_for_controller = joint_names_for_controller.index(joint_name)
            end_positions[joint_idx_for_controller] = joint_angle

            start_point = JointTrajectoryPoint(positions=current_commanded_positions_for_controller)
            end_point = JointTrajectoryPoint()
            end_point.positions = end_positions
            end_point.time_from_start.sec = 0
            end_point.time_from_start.nanosec = 100_000_000

            traj_msg = JointTrajectory()
            traj_msg.joint_names = joint_names_for_controller
            traj_msg.points = [start_point, end_point]

            print("waiting to create pub...")

            jtc_pub = self.side.get_jtc_cmd_pub(active_controller.name)
            print("Got publisher")
            jtc_pub.publish(traj_msg)

        elif active_controller.type == 'victor_hardware/KukaJointGroupPositionController':
            joint_cmd_msg = self.get_float64_from_sliders()
            joint_cmd_pub = self.side.get_joint_cmd_pub(active_controller.name)
            # print(len(joint_cmd_msg))
            self.side.send_joint_cmd(joint_cmd_pub, joint_cmd_msg.data)
        else:
            print(f"Unknown controller type {active_controller.type}")

    def publish_gripper_cmd(self):
        self.side.gripper_command.publish(self.gripper_cmd)

    def on_open_gripper(self):
        self.gripper_cmd = get_gripper_closed_fraction_msg(ROBOTIQ_OPEN)
        self.publish_gripper_cmd()

    def on_close_gripper(self):
        self.gripper_cmd = get_gripper_closed_fraction_msg(ROBOTIQ_CLOSED)
        self.publish_gripper_cmd()

    def get_float64_from_sliders(self):
        msg = Float64MultiArray()
        msg.data = [slider_pos_to_joint_angle(slider_widget.get_value()) for slider_widget in self.slider_widgets]
        return msg


class SliderWidget(QWidget):

    def __init__(self, label: str):
        QWidget.__init__(self)
        uic.loadUi(ui_dir / 'slider_widget.ui', self)
        self.label.setText(label)

    def connect(self):
        self.slider.valueChanged.connect(self.slider_changed)
        self.value_edit.editingFinished.connect(self.value_edit_changed)

    def value_edit_changed(self):
        text = self.value_edit.text()
        print(f'value edit changed: {text}')
        self.slider.setValue(int(text))

    def slider_changed(self, value):
        print(f"slider changed: {value}")
        self.value_edit.setText(str(value))

    def get_value(self):
        return self.slider.value()

    def set_value(self, value):
        self.slider.setValue(value)
        self.value_edit.setText(str(value))


class ArmJointSliderWidget(SliderWidget):
    jointChanged = pyqtSignal(int, float)

    def __init__(self, label: str, joint_idx: int):
        SliderWidget.__init__(self, label)
        self.joint_idx = joint_idx

    def value_edit_changed(self):
        super().value_edit_changed()

    def slider_changed(self, value):
        super().slider_changed(value)
        self.jointChanged.emit(self.joint_idx, slider_pos_to_joint_angle(value))


class FingerSliderWidget(SliderWidget):

    def __init__(self, label: str):
        SliderWidget.__init__(self, label)

    def value_edit_changed(self):
        text = self.value_edit.text()
        try:
            fraction = self.fraction_to_slider_pos(float(text))
            self.slider.setValue(int(fraction))
        except ValueError:
            print(f"Invalid float: {text}")

    def slider_changed(self, value):
        self.value_edit.setText(str(self.slider_pos_to_fraction(value)))

    def set_value(self, value):
        self.slider.setValue(value)
        self.value_edit.setText(str(self.slider_pos_to_fraction(value)))

    @staticmethod
    def fraction_to_slider_pos(position):
        return int(position * 100)

    @staticmethod
    def slider_pos_to_fraction(slider_pos):
        return slider_pos / 100


class FingerWidget(QWidget):
    # Define a custom signal
    cmdChanged = pyqtSignal(Robotiq3FingerActuatorCommand)

    def __init__(self, label: str):
        QWidget.__init__(self)

        self.finger_cmd = Robotiq3FingerActuatorCommand()

        # The finger widget itself is a vertical layout
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)

        self.position_slider_widget = FingerSliderWidget(f"{label} Position")
        self.speed_slider_widget = FingerSliderWidget(f"{label} Speed")
        self.force_slider_widget = FingerSliderWidget(f"{label} Force")

        self.position_slider_widget.slider.setRange(0, 100)
        self.speed_slider_widget.slider.setRange(0, 100)
        self.force_slider_widget.slider.setRange(0, 100)

        self.layout.addWidget(self.position_slider_widget)
        self.layout.addWidget(self.speed_slider_widget)
        self.layout.addWidget(self.force_slider_widget)

        self.position_slider_widget.connect()
        self.speed_slider_widget.connect()
        self.force_slider_widget.connect()

        self.position_slider_widget.slider.valueChanged.connect(self.on_position_change)
        self.speed_slider_widget.slider.valueChanged.connect(self.on_speed_change)
        self.force_slider_widget.slider.valueChanged.connect(self.on_force_change)

        self.speed_slider_widget.set_value(100)
        self.force_slider_widget.set_value(100)

    def on_position_change(self, value):
        self.finger_cmd.position = FingerSliderWidget.slider_pos_to_fraction(value)
        self.cmdChanged.emit(self.finger_cmd)

    def on_speed_change(self, value):
        self.finger_cmd.speed = FingerSliderWidget.slider_pos_to_fraction(value)
        self.cmdChanged.emit(self.finger_cmd)

    def on_force_change(self, value):
        self.finger_cmd.force = FingerSliderWidget.slider_pos_to_fraction(value)
        self.cmdChanged.emit(self.finger_cmd)

    def set_status(self, status: Robotiq3FingerActuatorStatus):
        self.position_slider_widget.set_value(FingerSliderWidget.fraction_to_slider_pos(status.position))


def main():
    rclpy.init()
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    node = Node('victor_command_gui')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    app = QApplication(sys.argv)

    widget = VictorCommandWindow(node)
    widget.showMaximized()

    # use a QTimer to call spin regularly
    def _spin_once():
        # print(".", end="")
        executor.spin_once()

    timer = QTimer()
    timer.timeout.connect(_spin_once)
    timer.start(10)

    exit_code = app.exec_()

    executor.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
