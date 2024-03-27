import rclpy
from rclpy.node import Node

from victor_hardware_interfaces.msg import MotionStatus, Robotiq3FingerStatus, Robotiq3FingerCommand


class MockVictorROS(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.left_motion_status_pub = self.create_publisher(MotionStatus, '/victor/left_arm/motion_status', 10)
        self.right_motion_status_pub = self.create_publisher(MotionStatus, '/victor/right_arm/motion_status', 10)
        self.left_gripper_status_pub = self.create_publisher(Robotiq3FingerStatus, '/victor/left_arm/gripper_status',
                                                             10)
        self.right_gripper_status_pub = self.create_publisher(Robotiq3FingerStatus, '/victor/right_arm/gripper_status',
                                                              10)

        self.left_gripper_cmd_sub = self.create_subscription(Robotiq3FingerCommand, '/victor/left_arm/gripper_command',
                                                             self.sub_cb, 10)

        self.status_timer = self.create_timer(0.01, self.status_timer_callback)

    def sub_cb(self, msg):
        pass

    def status_timer_callback(self):
        self.left_gripper_status_pub.publish(Robotiq3FingerStatus())
        self.right_gripper_status_pub.publish(Robotiq3FingerStatus())
        self.left_motion_status_pub.publish(MotionStatus())
        self.right_motion_status_pub.publish(MotionStatus())


def main(args=None):
    rclpy.init(args=args)
    v = MockVictorROS()
    rclpy.spin(v)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
