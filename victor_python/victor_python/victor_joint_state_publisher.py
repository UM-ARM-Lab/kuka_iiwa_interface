#!/usr/bin/env python3

from threading import Lock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from victor_python.victor import Victor


class VictorJointStatePublisher(Node):
    def __init__(self):
        super().__init__("victor_joint_state_publisher")

        # Set up the output message with default values
        self.joint_state_lock = Lock()

        # Set up the publishers and subscribers that will be used
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.victor = Victor(self)

        # Create timer to publish joint states at 100Hz
        self.create_timer(0.01, self.publish_joint_values)

    def publish_joint_values(self):
        with self.joint_state_lock:
            joint_state_msg = self.victor.get_measured_joint_states_from_status(include_gripper=True)
            self.joint_state_pub.publish(joint_state_msg)


def main():
    rclpy.init()
    node = VictorJointStatePublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
