# Simple ros node that sends TwistStamped messages to move the EE forward and backward

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
        # self.js_listener = Listener(self, '/joint_states', JointState)

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
        self.msg.joint_names = ["victor_left_arm_joint_7"]

        self.pub_idx = 0

        # timer callback
        self.timer = self.create_timer(0.01, self.timer_callback)


    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()

        # js = self.js_listener.get()

        # if js is None:
        #     print("No joint states received yet")
        #     return

        # ordered_names = [
        #     "victor_left_arm_joint_1",
        #     "victor_left_arm_joint_2",
        #     "victor_left_arm_joint_3",
        #     "victor_left_arm_joint_4",
        #     "victor_left_arm_joint_5",
        #     "victor_left_arm_joint_6",
        #     "victor_left_arm_joint_7",
        #     "victor_right_arm_joint_1",
        #     "victor_right_arm_joint_2",
        #     "victor_right_arm_joint_3",
        #     "victor_right_arm_joint_4",
        #     "victor_right_arm_joint_5",
        #     "victor_right_arm_joint_6",
        #     "victor_right_arm_joint_7",
        # ]
        # ordered_positions = [js.position[js.name.index(name)] for name in ordered_names]
        # msg = Float64MultiArray()
        # msg.data = ordered_positions
        # msg.data[6] += 0.5
        # print(msg)
        # self.publisher.publish(msg)

        # if self.pub_idx < 200:
        #     self.msg.twist.linear.y = 0.03
        #     self.publisher.publish(self.msg)
        # elif self.pub_idx < 400:
        if self.pub_idx < 400:
            # self.msg.twist.linear.y = 0.05
            self.msg.velocities = [-0.18] 
            self.publisher.publish(self.msg)
        else:
            # self.msg.twist.linear.y = 0.
            self.msg.velocities = [0.0] 
            self.publisher.publish(self.msg)
            print("done!")
            rclpy.shutdown()
        
        self.pub_idx += 1

def main():
    rclpy.init()

    node = TestServo()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
