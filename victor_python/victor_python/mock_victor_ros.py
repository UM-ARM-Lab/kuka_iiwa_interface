import rclpy
from rclpy.node import Node

from victor_hardware_interfaces.msg import MotionStatus, Robotiq3FingerStatus, Robotiq3FingerCommand
from victor_hardware_interfaces.srv import SetControlMode, GetControlMode


class MockVictorROS(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.left_set_srv = self.create_service(SetControlMode, '/victor/left_arm/set_control_mode_service',
                                                self.srv_cb)
        self.right_set_srv = self.create_service(SetControlMode, '/victor/right_arm/set_control_mode_service',
                                                 self.srv_cb)
        self.left_get_srv = self.create_service(GetControlMode, '/victor/left_arm/get_control_mode_service',
                                                self.srv_cb)
        self.right_get_srv = self.create_service(GetControlMode, '/victor/right_arm/get_control_mode_service',
                                                 self.srv_cb)

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

    def srv_cb(self, request, response):
        print(f"Received request: {request}")
        return response


def main(args=None):
    rclpy.init(args=args)
    v = MockVictorROS()
    rclpy.spin(v)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
