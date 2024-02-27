# Simple ros node that sends TwistStamped messages to move the EE forward and backward
from math import sin

import time
import rclpy
from rclpy.node import Node
from arm_utilities.listener import Listener
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class TestServo(Node):
    def __init__(self):
        super().__init__('test_servo')
        # self.publisher = self.create_publisher(TwistStamped, '/moveit_servo/delta_twist_cmds', 10)
        self.publisher = self.create_publisher(JointJog, '/moveit_servo/delta_joint_cmds', 10)
        # self.publisher = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)
        # self.js_listener = Listener(self, JointState, '/joint_states')

        # self.msg = TwistStamped()
        # self.msg.header.frame_id = "victor_left_palm"

        # # every 0.1 second publish a small +x velocity
        # self.msg.twist.linear.x = 0.0
        # self.msg.twist.linear.y = 0.0
        # self.msg.twist.linear.z = 0.0
        # self.msg.twist.angular.x = 0.0
        # self.msg.twist.angular.y = 0.0
        # self.msg.twist.angular.z = 0.0

        self.msg = JointJog()
        self.msg.joint_names = ["victor_left_arm_joint_6"]

        self.pub_idx = 0

        # timer callback
        self.timer = self.create_timer(0.01, self.timer_callback)


    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.msg.velocities = [2.5 * sin(self.pub_idx/100.0)]
        self.publisher.publish(self.msg)
        
        self.pub_idx += 1

def main():
    rclpy.init()

    node = TestServo()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
