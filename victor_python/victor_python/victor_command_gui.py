#!/usr/bin/env python3
import signal
import sys
import threading
from pathlib import Path
from typing import Dict

import numpy as np
from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import Robot
from victor_hardware_interfaces.msg import Robotiq3FingerCommand, Robotiq3FingerStatus, \
    Robotiq3FingerActuatorStatus, Robotiq3FingerActuatorCommand
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

        self.left_arm_widget = ArmWidget(node, self.victor, 'left_arm', self.victor.left)
        self.right_arm_widget = ArmWidget(node, self.victor, 'right_arm', self.victor.right)

        self.left_motion_group.layout().addWidget(self.left_arm_widget)
        self.right_motion_group.layout().addWidget(self.right_arm_widget)

    def robot_description_callback(self, robot: Robot):
        # NOTE: this callback gets called on its own thread, so don't directly call functions on QtWidgets here.
        self.left_arm_widget.on_robot_description(robot)
        self.right_arm_widget.on_robot_description(robot)


class ArmWidget(QWidget):
    initial_robot_data = pyqtSignal(Robot, dict, Robotiq3FingerStatus)
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
        for joint_idx in range(7):
            slider_widget = SliderWidget(f"Joint {joint_idx + 1} Command")
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

        active_controller_names = self.side.list_active_controllers()

        if len(active_controller_names) == 0:
            self.activeControllerChanged.emit("No active controllers", "", active_control_mode)
        elif len(active_controller_names) > 1:
            self.activeControllerChanged.emit("Multiple active controllers!!! Probably a bug...", "", active_control_mode)
        else:
            active_controller_name = active_controller_names[0]
            control_mode = self.side.get_control_mode_for_controller(active_controller_name)

            self.activeControllerChanged.emit(active_controller_name, control_mode, active_control_mode)

    def on_robot_description(self, robot_description: Robot):
        """ Called off the main thread """
        # Initialize the command objects to the current state
        joint_states_dict = self.victor.get_joint_cmd_dict()
        gripper_status: Robotiq3FingerStatus = self.side.gripper_status.get()

        # Emit back to the main thread
        self.initial_robot_data.emit(robot_description, joint_states_dict, gripper_status)

    def on_initial_robot_data(self,
                              robot: Robot,
                              joint_states_dict: Dict,
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
            slider_widget = self.slider_widgets[joint_idx]
            slider_widget.setEnabled(True)
            slider_widget.connect()
            slider_widget.slider.valueChanged.connect(self.publish_arm_cmd_async)

    def reset_sliders(self):
        # Read the current joint states and update the sliders
        joint_states_dict = self.victor.get_joint_cmd_dict()
        for joint_idx in range(7):
            expected_joint_name = f"victor_{self.arm_name}_joint_{joint_idx + 1}"
            current_rad = joint_states_dict[expected_joint_name]
            slider_pos = joint_angle_to_slider_pos(current_rad)
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

    def publish_arm_cmd_async(self):
        thread = threading.Thread(target=self.publish_arm_cmd)
        thread.start()

    def publish_arm_cmd(self):
        self.side.send_joint_cmd(self.get_float64_from_sliders())

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
        self.value_edit.textChanged.connect(self.value_edit_changed)

    def value_edit_changed(self, text):
        self.slider.setValue(int(text))

    def slider_changed(self, value):
        self.value_edit.setText(str(value))

    def get_value(self):
        return self.slider.value()

    def set_value(self, value):
        self.slider.setValue(value)
        self.value_edit.setText(str(value))


class FingerSliderWidget(SliderWidget):

    def __init__(self, label: str):
        SliderWidget.__init__(self, label)

    def value_edit_changed(self, text):
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
        executor.spin_once()

    timer = QTimer()
    timer.timeout.connect(_spin_once)
    timer.start(10)

    exit_code = app.exec_()

    executor.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
