# Simple ros node that sends TwistStamped messages to move the EE forward and backward
from math import sin

import time
import rclpy
from rclpy.node import Node
from victor_hardware_interfaces.msg import MotionCommand
from victor_hardware_interfaces.msg import ControlMode

class TestCartesianCommand(Node):
    def __init__(self):
        super().__init__('test_servo')
        self.publisher = self.create_publisher(MotionCommand, 'victor/left_arm/motion_command', 10)

        self.msg = MotionCommand()
        self.msg.control_mode = ControlMode.CARTESIAN_IMPEDANCE

        self.pub_idx = 0

        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()

        

        self.publisher.publish(self.msg)
        
        self.pub_idx += 1

def main():
    rclpy.init()

    node = TestCartesianCommand()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
