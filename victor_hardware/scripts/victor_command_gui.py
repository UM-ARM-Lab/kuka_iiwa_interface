#!/usr/bin/env python3
import threading

import signal
import sys
from pathlib import Path

import numpy as np
import rclpy
from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QTreeWidgetItem, QTreeWidget, \
    QHeaderView, QSpinBox, QDoubleSpinBox, QCheckBox, QLineEdit
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from urdf_parser_py.urdf import Robot
from urdf_parser_py.xml_reflection import core

from victor_hardware.victor import Victor, Side, ROBOTIQ_OPEN, ROBOTIQ_CLOSED
from victor_hardware.victor_utils import list_to_jvq, Stiffness, get_control_mode_params, \
    get_gripper_closed_fraction_msg
from victor_hardware_interfaces.msg import Robotiq3FingerCommand, MotionCommand, ControlMode, Robotiq3FingerStatus, \
    Robotiq3FingerActuatorStatus, Robotiq3FingerActuatorCommand, ControlModeParameters
from victor_hardware_interfaces.srv import GetControlMode, SetControlMode

# Suppress error messages from urdf_parser_py
core.on_error = lambda *args: None

ui_dir = Path(__file__).resolve().parent

name_to_control_mode_map = {
    "Joint Position": ControlMode.JOINT_POSITION,
    "Joint Impedance": ControlMode.JOINT_IMPEDANCE,
    "Cartesian Impedance": ControlMode.CARTESIAN_IMPEDANCE,
}
control_mode_to_name_map = {v: k for k, v in name_to_control_mode_map.items()}


def joint_angle_to_slider_pos(pos):
    return int(np.rad2deg(pos))


def slider_pos_to_joint_angle(slider_pos):
    return np.deg2rad(slider_pos)


def tree_iterator(tree: QTreeWidget):
    def _tree_iterator(item: QTreeWidgetItem):
        for i in range(item.childCount()):
            child_item = item.child(i)
            yield item
            yield from _tree_iterator(child_item)

    for i in range(tree.topLevelItemCount()):
        item = tree.topLevelItem(i)
        yield item
        yield from _tree_iterator(item)


class VictorCommandWindow(QMainWindow):
    def __init__(self, node: Node, parent=None):
        QMainWindow.__init__(self, parent)
        uic.loadUi(ui_dir / 'victor_command_gui.ui', self)

        self.victor = Victor(node, robot_description_cb=self.robot_description_callback)

        self.arm_cmd_pub = self.victor.left_arm_cmd_pub

        self.left_arm = ArmWidget(node, self.victor, 'left_arm', self.victor.left)
        self.right_arm = ArmWidget(node, self.victor, 'right_arm', self.victor.right)

        self.left_motion_group.layout().addWidget(self.left_arm)
        self.right_motion_group.layout().addWidget(self.right_arm)

        self.left_params_widget = ControlModeParamsWidget(node, self.victor.left)
        self.right_params_widget = ControlModeParamsWidget(node, self.victor.right)

        self.left_params_group.layout().addWidget(self.left_params_widget)
        self.right_params_group.layout().addWidget(self.right_params_widget)

    def robot_description_callback(self, robot: Robot):
        # NOTE: this callback gets called on its own thread, so don't directly call functions on QtWidgets here.
        self.left_arm.on_robot_description(robot)
        self.right_arm.on_robot_description(robot)


class ArmWidget(QWidget):

    def __init__(self, node: Node, victor: Victor, arm_name: str, side: Side):
        QWidget.__init__(self)
        uic.loadUi(ui_dir / 'arm_widget.ui', self)
        self.node = node
        self.victor = victor
        self.arm_name = arm_name
        self.side = side

        # Create the command objects which we will update in the callbacks and publish
        self.gripper_cmd = Robotiq3FingerCommand()
        self.motion_cmd = MotionCommand()
        self.stiffness = Stiffness.MEDIUM

        # Add slider widgets for each joint, but we don't have values yet
        self.slider_widgets = []
        for joint_idx in range(7):
            slider_widget = SliderWidget(f"Joint {joint_idx + 1} Command")
            slider_widget.setEnabled(False)
            self.slider_widgets.append(slider_widget)
            self.arm_layout.addWidget(slider_widget)

        self.control_mode_combo.addItems(name_to_control_mode_map.keys())
        self.stiffness_combo.addItems([str(stiffness) for stiffness in Stiffness])

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

        # Start things as disabled, since we have to wait for the robot description to enable them
        self.reset_sliders_button.setEnabled(False)
        self.control_mode_combo.setEnabled(False)
        self.stiffness_combo.setEnabled(False)

    def on_robot_description(self, robot_description: Robot):
        # Initialize the command objects to the current state
        print("Arm on_robot_description")
        joint_states = self.victor.get_joint_positions_dict()
        print("got joint positions")

        client = self.side.get_control_mode
        future = client.call_async(GetControlMode.Request())
        rclpy.spin_until_future_complete(self.node, future)
        active_control_mode_res: ControlModeParameters = future.result()

        self.motion_cmd.control_mode = active_control_mode_res.active_control_mode.control_mode

        joint_positions_list = []
        for joint_idx in range(7):
            expected_joint_name = f"victor_{self.arm_name}_joint_{joint_idx + 1}"
            joint = robot_description.joint_map[expected_joint_name]
            lower_deg = joint_angle_to_slider_pos(joint.limit.lower)
            upper_deg = joint_angle_to_slider_pos(joint.limit.upper)

            current_rad = joint_states[expected_joint_name]
            joint_positions_list.append(current_rad)

            slider_pos = joint_angle_to_slider_pos(current_rad)

            slider_widget = self.slider_widgets[joint_idx]
            # Set the range and current position of the joint from the current joint state
            slider_widget.slider.setRange(lower_deg, upper_deg)
            slider_widget.set_value(slider_pos)

        # Set the values of the gripper sliders
        gripper_status: Robotiq3FingerStatus = self.side.gripper_status.get()
        self.finger_a_widget.set_status(gripper_status.finger_a_status)
        self.finger_b_widget.set_status(gripper_status.finger_b_status)
        self.finger_c_widget.set_status(gripper_status.finger_c_status)

        self.motion_cmd.joint_position = list_to_jvq(joint_positions_list)

        # Set the control mode
        control_mode_name = control_mode_to_name_map[active_control_mode_res.active_control_mode.control_mode.mode]
        self.control_mode_combo.setCurrentText(control_mode_name)

        # Connect combo box
        self.control_mode_combo.activated.connect(self.change_control_mode)
        self.stiffness_combo.activated.connect(self.set_stiffness)

        # Enable things
        self.reset_sliders_button.setEnabled(True)
        self.control_mode_combo.setEnabled(True)
        self.stiffness_combo.setEnabled(True)

        # Connect the sliders
        for joint_idx in range(7):
            slider_widget = self.slider_widgets[joint_idx]
            slider_widget.setEnabled(True)
            slider_widget.connect()
            slider_widget.slider.valueChanged.connect(self.publish_arm_cmd)

    def reset_sliders(self):
        # Read the current joint states and update the sliders
        joint_states = self.victor.get_joint_positions_dict()
        for joint_idx in range(7):
            expected_joint_name = f"victor_{self.arm_name}_joint_{joint_idx + 1}"
            current_rad = joint_states[expected_joint_name]
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

    def publish_arm_cmd(self):
        self.motion_cmd.joint_position = self.get_jvq_from_sliders()
        self.side.motion_command.publish(self.motion_cmd)

    def publish_gripper_cmd(self):
        self.side.gripper_command.publish(self.gripper_cmd)

    def on_open_gripper(self):
        self.gripper_cmd = get_gripper_closed_fraction_msg(ROBOTIQ_OPEN)
        self.publish_gripper_cmd()

    def on_close_gripper(self):
        self.gripper_cmd = get_gripper_closed_fraction_msg(ROBOTIQ_CLOSED)
        self.publish_gripper_cmd()

    def get_jvq_from_sliders(self):
        return list_to_jvq(
            [slider_pos_to_joint_angle(slider_widget.get_value()) for slider_widget in self.slider_widgets])

    def change_control_mode(self, item_idx: int):
        control_mode = ControlMode()
        control_mode.mode = name_to_control_mode_map[self.control_mode_combo.itemText(item_idx)]
        self.send_change_control_mode(control_mode)

    def set_stiffness(self, item_idx: int):
        self.stiffness = Stiffness(item_idx)
        self.send_change_control_mode(self.motion_cmd.control_mode)

    def send_change_control_mode(self, control_mode: ControlMode):
        request = SetControlMode.Request()
        request.new_control_mode = get_control_mode_params(control_mode.mode, self.stiffness)

        client = self.side.set_control_mode
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        future.result()


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

        # The finger widget itself is a vertical layout
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)

        self.position_slider_widget = FingerSliderWidget(f"{label} Position")
        self.speed_slider_widget = FingerSliderWidget(f"{label} Speed")
        self.force_slider_widget = FingerSliderWidget(f"{label} Force")

        self.position_slider_widget.slider.setRange(0, 100)
        self.speed_slider_widget.slider.setRange(0, 100)
        self.speed_slider_widget.set_value(100)
        self.force_slider_widget.slider.setRange(0, 100)
        self.force_slider_widget.set_value(100)


        self.layout.addWidget(self.position_slider_widget)
        self.layout.addWidget(self.speed_slider_widget)
        self.layout.addWidget(self.force_slider_widget)

        self.position_slider_widget.connect()
        self.speed_slider_widget.connect()
        self.force_slider_widget.connect()

        self.position_slider_widget.slider.valueChanged.connect(self.on_position_change)
        self.speed_slider_widget.slider.valueChanged.connect(self.on_speed_change)
        self.force_slider_widget.slider.valueChanged.connect(self.on_force_change)

        self.finger_cmd = Robotiq3FingerActuatorCommand()

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


class ControlModeParamsWidget(QWidget):

    def __init__(self, node: Node, side):
        QWidget.__init__(self)
        uic.loadUi(ui_dir / 'control_mode_params_widget.ui', self)
        self.node = node
        self.side = side

        self.send_button.clicked.connect(self.send_params)

        # auto-resize the tree columns
        self.params_tree.header().setSectionResizeMode(QHeaderView.ResizeToContents)

        self.reset_params_tree()

    def reset_params_tree(self):
        self.params_tree.clear()

        print("init params widget", threading.get_ident())
        client = self.side.get_control_mode
        future = client.call_async(GetControlMode.Request())
        rclpy.spin_until_future_complete(self.node, future)
        active_control_mode: ControlModeParameters = future.result().active_control_mode

        populate_tree_from_msg(self.params_tree, active_control_mode)

        # expand all items
        self.params_tree.expandAll()

    def send_params(self):
        req = SetControlMode.Request()
        tree_to_control_mode_params(self.params_tree, req.new_control_mode)

        client = self.side.set_control_mode
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        future.result()


def tree_item_to_control_mode_params(tree: QTreeWidget, item: QTreeWidgetItem, msg):
    for i in range(item.childCount()):
        child_item = item.child(i)
        field_name = child_item.text(0)
        field = getattr(msg, field_name)
        recurse = item_to_control_mode_params(tree, child_item, msg, field_name)
        if recurse:
            tree_item_to_control_mode_params(tree, child_item, field)


def item_to_control_mode_params(tree: QTreeWidget, item: QTreeWidgetItem, msg, field_name: str):
    widget = tree.itemWidget(item, 1)
    if isinstance(widget, QDoubleSpinBox):
        setattr(msg, field_name, widget.value())
    elif isinstance(widget, QSpinBox):
        setattr(msg, field_name, widget.value())
    elif isinstance(widget, QLineEdit):
        setattr(msg, field_name, widget.text())
    elif isinstance(widget, QCheckBox):
        setattr(msg, field_name, widget.isChecked())
    else:
        return True
    return False


def tree_to_control_mode_params(tree: QTreeWidget, msg: ControlModeParameters):
    # iterate over the top level items
    for i in range(tree.topLevelItemCount()):
        item = tree.topLevelItem(i)
        field_name = item.text(0)
        field = getattr(msg, field_name)

        recurse = item_to_control_mode_params(tree, item, field, field_name)
        if recurse:
            tree_item_to_control_mode_params(tree, item, field)


# FIXME: get this from rosidl_parser?

def populate_tree_from_msg(tree: QTreeWidget, msg):
    def _populate_items(item: QTreeWidgetItem, msg):
        for field_name, field_type_str in msg.get_fields_and_field_types().items():
            field = getattr(msg, field_name)

            if field_type_str in ['uint8', 'int32', 'uint32']:
                child_item = QTreeWidgetItem([field_name])
                item.addChild(child_item)
                item_widget = QSpinBox()
                item_widget.wheelEvent = lambda event: None  # disable mouse wheel
                item_widget.setValue(int(field))
                tree.setItemWidget(child_item, 1, item_widget)
            elif field_type_str == 'double':
                child_item = QTreeWidgetItem([field_name])
                item.addChild(child_item)
                item_widget = QDoubleSpinBox()
                item_widget.wheelEvent = lambda event: None  # disable mouse wheel
                item_widget.setRange(-1e6, 1e6)  # don't want to limit the range
                item_widget.setValue(float(field))
                tree.setItemWidget(child_item, 1, item_widget)
            elif field_type_str == 'string':
                child_item = QTreeWidgetItem([field_name])
                item.addChild(child_item)
                item_widget = QLineEdit()
                item_widget.setText(str(field))
                tree.setItemWidget(child_item, 1, item_widget)
            elif field_type_str == 'boolean':
                child_item = QTreeWidgetItem([field_name])
                item.addChild(child_item)
                item_widget = QCheckBox()
                item_widget.setChecked(bool(field))
                tree.setItemWidget(child_item, 1, item_widget)
            else:
                child_item = QTreeWidgetItem([field_name])
                item.addChild(child_item)
                _populate_items(child_item, field)

    # Create the top-level items
    top_level_items = []
    for field_name, field_type_str in msg.get_fields_and_field_types().items():
        field = getattr(msg, field_name)

        top_level_item = QTreeWidgetItem([field_name])
        top_level_items.append(top_level_item)

        if is_value_type_str(field_type_str):
            top_level_item.addChild(QTreeWidgetItem([str(field)]))
        else:
            _populate_items(top_level_item, field)

        top_level_item.setExpanded(True)

    tree.insertTopLevelItems(0, top_level_items)


def is_value_type_str(field_type_str):
    return field_type_str in ['uint8', 'int32', 'uint32', 'double', 'string', 'boolean']

def my_spin(executor):
    while True:
        print("spinning...", threading.get_ident())
        executor.spin_once()


def main():
    rclpy.init()
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    node = Node('victor_command_gui')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    app = QApplication(sys.argv)

    widget = VictorCommandWindow(node)
    widget.showMaximized()

    spin_thread = threading.Thread(target=my_spin, args=(executor,))
    spin_thread.start()

    exit_code = app.exec_()

    executor.shutdown()

    spin_thread.join()

    sys.exit(exit_code)



if __name__ == "__main__":
    main()
