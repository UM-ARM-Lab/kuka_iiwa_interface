#!/usr/bin/env python

# [Create a window]

import sys
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rospkg
import rospy
import math
from wiktor_hardware_interface.msg import Robotiq3FingerCommand, MotionCommand

finger_range_discretization = 1000
arm_joint_limit_margin = 1

class Widget(QWidget):
	def __init__(self, parent=None):
			QWidget.__init__(self, parent)

			self.v_layout = QGridLayout()

			self.reset_arm_button = QPushButton('Home the Arm')
			self.v_layout.addWidget(self.reset_arm_button,0,0)

			self.open_gripper_button = QPushButton('Open the Gripper')
			self.v_layout.addWidget(self.open_gripper_button,0,2)

			self.close_gripper_button = QPushButton('Close the Gripper')
			self.v_layout.addWidget(self.close_gripper_button,1,2)

			self.finger_same_position_checkbox = QCheckBox('Move the fingers together')
			self.v_layout.addWidget(self.finger_same_position_checkbox,2,2)

			self.joint_1_label = QLabel('Joint 1 Command: 0')
			self.joint_1_slider = QSlider()
			self.joint_1_slider.setOrientation(Qt.Horizontal)
			self.joint_1_slider.setRange(-170+arm_joint_limit_margin,170-arm_joint_limit_margin)
			self.v_layout.addWidget(self.joint_1_label,1,0)
			self.v_layout.addWidget(self.joint_1_slider,2,0)

			self.joint_2_label = QLabel('Joint 2 Command: 0')
			self.joint_2_slider = QSlider()
			self.joint_2_slider.setOrientation(Qt.Horizontal)
			self.joint_2_slider.setRange(-120+arm_joint_limit_margin,120-arm_joint_limit_margin)
			self.v_layout.addWidget(self.joint_2_label,3,0)
			self.v_layout.addWidget(self.joint_2_slider,4,0)

			self.joint_3_label = QLabel('Joint 3 Command: 0')
			self.joint_3_slider = QSlider()
			self.joint_3_slider.setOrientation(Qt.Horizontal)
			self.joint_3_slider.setRange(-170+arm_joint_limit_margin,170-arm_joint_limit_margin)
			self.v_layout.addWidget(self.joint_3_label,5,0)
			self.v_layout.addWidget(self.joint_3_slider,6,0)

			self.joint_4_label = QLabel('Joint 4 Command: 0')
			self.joint_4_slider = QSlider()
			self.joint_4_slider.setOrientation(Qt.Horizontal)
			self.joint_4_slider.setRange(-120+arm_joint_limit_margin,120-arm_joint_limit_margin)
			self.v_layout.addWidget(self.joint_4_label,7,0)
			self.v_layout.addWidget(self.joint_4_slider,8,0)

			self.joint_5_label = QLabel('Joint 5 Command: 0')
			self.joint_5_slider = QSlider()
			self.joint_5_slider.setOrientation(Qt.Horizontal)
			self.joint_5_slider.setRange(-170+arm_joint_limit_margin,170-arm_joint_limit_margin)
			self.v_layout.addWidget(self.joint_5_label,9,0)
			self.v_layout.addWidget(self.joint_5_slider,10,0)

			self.joint_6_label = QLabel('Joint 6 Command: 0')
			self.joint_6_slider = QSlider()
			self.joint_6_slider.setOrientation(Qt.Horizontal)
			self.joint_6_slider.setRange(-120+arm_joint_limit_margin,120-arm_joint_limit_margin)
			self.v_layout.addWidget(self.joint_6_label,11,0)
			self.v_layout.addWidget(self.joint_6_slider,12,0)

			self.joint_7_label = QLabel('Joint 7 Command: 0')
			self.joint_7_slider = QSlider()
			self.joint_7_slider.setOrientation(Qt.Horizontal)
			self.joint_7_slider.setRange(-175+arm_joint_limit_margin,175-arm_joint_limit_margin)
			self.v_layout.addWidget(self.joint_7_label,13,0)
			self.v_layout.addWidget(self.joint_7_slider,14,0)

			self.joint_1_textbox = QLineEdit('0')
			self.joint_2_textbox = QLineEdit('0')
			self.joint_3_textbox = QLineEdit('0')
			self.joint_4_textbox = QLineEdit('0')
			self.joint_5_textbox = QLineEdit('0')
			self.joint_6_textbox = QLineEdit('0')
			self.joint_7_textbox = QLineEdit('0')

			self.v_layout.addWidget(self.joint_1_textbox,2,1)
			self.v_layout.addWidget(self.joint_2_textbox,4,1)
			self.v_layout.addWidget(self.joint_3_textbox,6,1)
			self.v_layout.addWidget(self.joint_4_textbox,8,1)
			self.v_layout.addWidget(self.joint_5_textbox,10,1)
			self.v_layout.addWidget(self.joint_6_textbox,12,1)
			self.v_layout.addWidget(self.joint_7_textbox,14,1)

			self.finger_a_label = QLabel('Finger A Command: 0')
			self.finger_a_slider = QSlider()
			self.finger_a_slider.setOrientation(Qt.Horizontal)
			self.finger_a_slider.setRange(0,finger_range_discretization)
			self.v_layout.addWidget(self.finger_a_label,3,2)
			self.v_layout.addWidget(self.finger_a_slider,4,2)

			self.finger_b_label = QLabel('Finger B Command: 0')
			self.finger_b_slider = QSlider()
			self.finger_b_slider.setOrientation(Qt.Horizontal)
			self.finger_b_slider.setRange(0,finger_range_discretization)
			self.v_layout.addWidget(self.finger_b_label,5,2)
			self.v_layout.addWidget(self.finger_b_slider,6,2)

			self.finger_c_label = QLabel('Finger C Command: 0')
			self.finger_c_slider = QSlider()
			self.finger_c_slider.setOrientation(Qt.Horizontal)
			self.finger_c_slider.setRange(0,finger_range_discretization)
			self.v_layout.addWidget(self.finger_c_label,7,2)
			self.v_layout.addWidget(self.finger_c_slider,8,2)

			self.scissor_label = QLabel('Scissor Command: 0')
			self.scissor_slider = QSlider()
			self.scissor_slider.setOrientation(Qt.Horizontal)
			self.scissor_slider.setRange(0,finger_range_discretization)
			self.v_layout.addWidget(self.scissor_label,9,2)
			self.v_layout.addWidget(self.scissor_slider,10,2)

			self.finger_a_textbox = QLineEdit('0.000')
			self.finger_b_textbox = QLineEdit('0.000')
			self.finger_c_textbox = QLineEdit('0.000')
			self.scissor_textbox = QLineEdit('0.000')
			self.v_layout.addWidget(self.finger_a_textbox,4,3)
			self.v_layout.addWidget(self.finger_b_textbox,6,3)
			self.v_layout.addWidget(self.finger_c_textbox,8,3)
			self.v_layout.addWidget(self.scissor_textbox,10,3)


			# # self.finger_a_textbox.move(20, 20)
			# self.finger_a_textbox.resize(40,40)

			self.setLayout(self.v_layout)

			self.open_gripper_button.clicked.connect(self.open_gripper_command)
			self.close_gripper_button.clicked.connect(self.close_gripper_command)
			self.finger_a_slider.valueChanged.connect(self.finger_a_slider_moved)
			self.finger_b_slider.valueChanged.connect(self.finger_b_slider_moved)
			self.finger_c_slider.valueChanged.connect(self.finger_c_slider_moved)
			self.scissor_slider.valueChanged.connect(self.scissor_slider_moved)

			self.reset_arm_button.clicked.connect(self.reset_arm_command)
			self.joint_1_slider.valueChanged.connect(self.joint_1_slider_moved)
			self.joint_2_slider.valueChanged.connect(self.joint_2_slider_moved)
			self.joint_3_slider.valueChanged.connect(self.joint_3_slider_moved)
			self.joint_4_slider.valueChanged.connect(self.joint_4_slider_moved)
			self.joint_5_slider.valueChanged.connect(self.joint_5_slider_moved)
			self.joint_6_slider.valueChanged.connect(self.joint_6_slider_moved)
			self.joint_7_slider.valueChanged.connect(self.joint_7_slider_moved)

			self.finger_same_position_checkbox.stateChanged.connect(self.finger_same_position_checkbox_changed)

			self.finger_a_textbox.editingFinished.connect(self.finger_a_textbox_modified)
			self.finger_b_textbox.editingFinished.connect(self.finger_b_textbox_modified)
			self.finger_c_textbox.editingFinished.connect(self.finger_c_textbox_modified)
			self.scissor_textbox.editingFinished.connect(self.scissor_textbox_modified)

			self.joint_1_textbox.editingFinished.connect(self.joint_1_textbox_modified)
			self.joint_2_textbox.editingFinished.connect(self.joint_2_textbox_modified)
			self.joint_3_textbox.editingFinished.connect(self.joint_3_textbox_modified)
			self.joint_4_textbox.editingFinished.connect(self.joint_4_textbox_modified)
			self.joint_5_textbox.editingFinished.connect(self.joint_5_textbox_modified)
			self.joint_6_textbox.editingFinished.connect(self.joint_6_textbox_modified)
			self.joint_7_textbox.editingFinished.connect(self.joint_7_textbox_modified)

			self.finger_command = Robotiq3FingerCommand()
			self.arm_command = MotionCommand()

			self.finger_command.finger_a_command.speed = 1.0
			self.finger_command.finger_b_command.speed = 1.0
			self.finger_command.finger_c_command.speed = 1.0
			self.finger_command.scissor_command.speed = 1.0

			self.finger_command_publisher = rospy.Publisher('gripper_command', Robotiq3FingerCommand, queue_size=1)
			self.arm_command_publisher = rospy.Publisher('motion_command', MotionCommand, queue_size=1)

			self.fingers_same_position = False

	def finger_same_position_checkbox_changed(self):
		if(self.finger_same_position_checkbox.isChecked()):
			self.fingers_same_position = True
		else:
			self.fingers_same_position = False

	def open_gripper_command(self):
		self.finger_a_slider.setValue(0)
		self.finger_b_slider.setValue(0)
		self.finger_c_slider.setValue(0)
		self.finger_command.finger_a_command.position = 0
		self.finger_command.finger_b_command.position = 0
		self.finger_command.finger_c_command.position = 0
		self.finger_command_publisher.publish(self.finger_command)

	def close_gripper_command(self):
		self.finger_a_slider.setValue(finger_range_discretization)
		self.finger_b_slider.setValue(finger_range_discretization)
		self.finger_c_slider.setValue(finger_range_discretization)
		self.finger_command.finger_a_command.position = 1
		self.finger_command.finger_b_command.position = 1
		self.finger_command.finger_c_command.position = 1
		self.finger_command_publisher.publish(self.finger_command)

	def reset_arm_command(self):
		self.joint_1_slider.setValue(0)
		self.joint_2_slider.setValue(0)
		self.joint_3_slider.setValue(0)
		self.joint_4_slider.setValue(0)
		self.joint_5_slider.setValue(0)
		self.joint_6_slider.setValue(0)
		self.joint_7_slider.setValue(0)

		self.arm_command.joint_position.joint_1 = 0
		self.arm_command.joint_position.joint_2 = 0
		self.arm_command.joint_position.joint_3 = 0
		self.arm_command.joint_position.joint_4 = 0
		self.arm_command.joint_position.joint_5 = 0
		self.arm_command.joint_position.joint_6 = 0
		self.arm_command.joint_position.joint_7 = 0

		self.arm_command.joint_velocity.joint_1 = 0
		self.arm_command.joint_velocity.joint_2 = 0
		self.arm_command.joint_velocity.joint_3 = 0
		self.arm_command.joint_velocity.joint_4 = 0
		self.arm_command.joint_velocity.joint_5 = 0
		self.arm_command.joint_velocity.joint_6 = 0
		self.arm_command.joint_velocity.joint_7 = 0

		self.arm_command_publisher.publish(self.arm_command)

	def move_all_fingers(self,position):
		value = float(position)/finger_range_discretization
		value = min(max(value,0),1)
		position = int(value*finger_range_discretization)
		self.finger_a_label.setText('Finger A Command: %5.3f' % value)
		self.finger_a_textbox.setText('%5.3f'%value)
		self.finger_command.finger_a_command.position = value
		self.finger_command.finger_a_command.speed = 1.0
		self.finger_a_slider.setValue(position)

		self.finger_b_label.setText('Finger B Command: %5.3f' % value)
		self.finger_b_textbox.setText('%5.3f'%value)
		self.finger_command.finger_b_command.position = value
		self.finger_command.finger_b_command.speed = 1.0
		self.finger_b_slider.setValue(position)

		self.finger_c_label.setText('Finger C Command: %5.3f' % value)
		self.finger_c_textbox.setText('%5.3f'%value)
		self.finger_command.finger_c_command.position = value
		self.finger_command.finger_c_command.speed = 1.0
		self.finger_c_slider.setValue(position)

		self.finger_command_publisher.publish(self.finger_command)

	def finger_a_textbox_modified(self):
		self.finger_a_slider_moved(int(float(self.finger_a_textbox.displayText())*finger_range_discretization))

	def finger_b_textbox_modified(self):
		self.finger_b_slider_moved(int(float(self.finger_b_textbox.displayText())*finger_range_discretization))

	def finger_c_textbox_modified(self):
		self.finger_c_slider_moved(int(float(self.finger_c_textbox.displayText())*finger_range_discretization))

	def scissor_textbox_modified(self):
		self.scissor_slider_moved(int(float(self.scissor_textbox.displayText())*finger_range_discretization))

	def finger_a_slider_moved(self, position):
		if(self.fingers_same_position):
			self.move_all_fingers(position)
		else:
			value = float(position)/finger_range_discretization
			value = min(max(value,0),1)
			position = int(value*finger_range_discretization)
			self.finger_a_label.setText('Finger A Command: %5.3f' % value)
			self.finger_a_textbox.setText('%5.3f'%value)
			self.finger_a_slider.setValue(position)
			self.finger_command.finger_a_command.position = value
			self.finger_command.finger_a_command.speed = 1.0
			self.finger_command.finger_a_command.force = 1.0
			self.finger_command_publisher.publish(self.finger_command)

	def finger_b_slider_moved(self, position):
		if(self.fingers_same_position):
			self.move_all_fingers(position)
		else:
			value = float(position)/finger_range_discretization
			value = min(max(value,0),1)
			position = int(value*finger_range_discretization)
			self.finger_b_label.setText('Finger B Command: %5.3f' % value)
			self.finger_b_textbox.setText('%5.3f'%value)
			self.finger_b_slider.setValue(position)
			self.finger_command.finger_b_command.position = value
			self.finger_command.finger_b_command.speed = 1.0
			self.finger_command.finger_b_command.force = 1.0
			self.finger_command_publisher.publish(self.finger_command)

	def finger_c_slider_moved(self, position):
		if(self.fingers_same_position):
			self.move_all_fingers(position)
		else:
			value = float(position)/finger_range_discretization
			value = min(max(value,0),1)
			position = int(value*finger_range_discretization)
			self.finger_c_label.setText('Finger C Command: %5.3f' % value)
			self.finger_c_textbox.setText('%5.3f'%value)
			self.finger_c_slider.setValue(position)
			self.finger_command.finger_c_command.position = value
			self.finger_command.finger_c_command.speed = 1.0
			self.finger_command.finger_c_command.force = 1.0
			self.finger_command_publisher.publish(self.finger_command)

	def scissor_slider_moved(self, position):
		value = float(position)/finger_range_discretization
		value = min(max(value,0),1)
		position = int(value*finger_range_discretization)
		self.scissor_label.setText('Scissor Command: %5.3f' % value)
		self.scissor_textbox.setText('%5.3f'%value)
		self.scissor_slider.setValue(position)
		self.finger_command.scissor_command.position = value
		self.finger_command.scissor_command.speed = 1.0
		self.finger_command.scissor_command.force = 1.0
		self.finger_command_publisher.publish(self.finger_command)

	def joint_1_textbox_modified(self):
		value = int(self.joint_1_textbox.displayText())
		value = min(max(-170+arm_joint_limit_margin,value),170-arm_joint_limit_margin,value)
		self.joint_1_slider_moved(value)

	def joint_2_textbox_modified(self):
		value = int(self.joint_2_textbox.displayText())
		value = min(max(-120+arm_joint_limit_margin,value),120-arm_joint_limit_margin,value)
		self.joint_2_slider_moved(value)

	def joint_3_textbox_modified(self):
		value = int(self.joint_3_textbox.displayText())
		value = min(max(-170+arm_joint_limit_margin,value),170-arm_joint_limit_margin,value)
		self.joint_3_slider_moved(value)

	def joint_4_textbox_modified(self):
		value = int(self.joint_4_textbox.displayText())
		value = min(max(-120+arm_joint_limit_margin,value),120-arm_joint_limit_margin,value)
		self.joint_4_slider_moved(value)

	def joint_5_textbox_modified(self):
		value = int(self.joint_5_textbox.displayText())
		value = min(max(-170+arm_joint_limit_margin,value),170-arm_joint_limit_margin,value)
		self.joint_5_slider_moved(value)

	def joint_6_textbox_modified(self):
		value = int(self.joint_6_textbox.displayText())
		value = min(max(-120+arm_joint_limit_margin,value),120-arm_joint_limit_margin,value)
		self.joint_6_slider_moved(value)

	def joint_7_textbox_modified(self):
		value = int(self.joint_7_textbox.displayText())
		value = min(max(-170+arm_joint_limit_margin,value),170-arm_joint_limit_margin,value)
		self.joint_7_slider_moved(value)

	def joint_1_slider_moved(self, position):
		self.joint_1_label.setText('Joint 1 Command: %d' % position)
		self.joint_1_textbox.setText(str(position))
		self.joint_1_slider.setValue(position)
		self.arm_command.joint_position.joint_1 = position * math.pi/180
		self.arm_command_publisher.publish(self.arm_command)

	def joint_2_slider_moved(self, position):
		self.joint_2_label.setText('Joint 2 Command: %d' % position)
		self.joint_2_textbox.setText(str(position))
		self.joint_2_slider.setValue(position)
		self.arm_command.joint_position.joint_2 = position * math.pi/180
		self.arm_command_publisher.publish(self.arm_command)

	def joint_3_slider_moved(self, position):
		self.joint_3_label.setText('Joint 3 Command: %d' % position)
		self.joint_3_textbox.setText(str(position))
		self.joint_3_slider.setValue(position)
		self.arm_command.joint_position.joint_3 = position * math.pi/180
		self.arm_command_publisher.publish(self.arm_command)

	def joint_4_slider_moved(self, position):
		self.joint_4_label.setText('Joint 4 Command: %d' % position)
		self.joint_4_textbox.setText(str(position))
		self.joint_4_slider.setValue(position)
		self.arm_command.joint_position.joint_4 = position * math.pi/180
		self.arm_command_publisher.publish(self.arm_command)

	def joint_5_slider_moved(self, position):
		self.joint_5_label.setText('Joint 5 Command: %d' % position)
		self.joint_5_textbox.setText(str(position))
		self.joint_5_slider.setValue(position)
		self.arm_command.joint_position.joint_5 = position * math.pi/180
		self.arm_command_publisher.publish(self.arm_command)

	def joint_6_slider_moved(self, position):
		self.joint_6_label.setText('Joint 6 Command: %d' % position)
		self.joint_6_textbox.setText(str(position))
		self.joint_6_slider.setValue(position)
		self.arm_command.joint_position.joint_6 = position * math.pi/180
		self.arm_command_publisher.publish(self.arm_command)

	def joint_7_slider_moved(self, position):
		self.joint_7_label.setText('Joint 7 Command: %d' % position)
		self.joint_7_textbox.setText(str(position))
		self.joint_7_slider.setValue(position)
		self.arm_command.joint_position.joint_7 = position * math.pi/180
		self.arm_command_publisher.publish(self.arm_command)

def main():

	rospy.init_node('arm_command_widget')

	app = QApplication(sys.argv)

	widget = Widget()
	widget.setWindowTitle("Arm Command Widget")
	widget.show()

	sys.exit(app.exec_())

if __name__ == "__main__":
		main()
