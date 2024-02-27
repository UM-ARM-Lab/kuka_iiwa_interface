import time

from victor_hardware_interfaces.msg import MotionCommand, MotionStatus
import numpy as np

import rclpy
from victor_hardware.victor import Victor, Side
from victor_hardware.victor_utils import jvq_to_list, list_to_jvq
from rclpy.node import Node


class ManualMotionFilter:

    def __init__(self, node: Node, side: Side, threshold: float = 0.04):
        """
        Keeps track of the current commanded and measured positions of the robot arms.
        If the commanded position is within a certain threshold of the measured position, we don't send the command.
        Otherwise, we command the arm to match the current measured position.
        """
        self.low_pass_tau = 0.005  # seconds
        self.node = node
        self.side = side
        self.threshold = threshold
        self.last_t = time.time()

    def update(self):
        now = time.time()
        dt = now - self.last_t
        if dt < 0.25:
            self.update_cmd(dt)
        else:
            print(f"{dt=} is too high! Something is wrong...")
        self.last_t = now

    def update_cmd(self, dt: float):
        status: MotionStatus = self.side.motion_status.get()

        measured_np = np.array(jvq_to_list(status.measured_joint_position))
        commanded_np = np.array(jvq_to_list(status.commanded_joint_position))
        delta = np.linalg.norm(measured_np - commanded_np)

        if delta < self.threshold:
            return
        
        # Low pass filter to avoid jerky motions
        alpha = dt / (self.low_pass_tau + dt)
        filtered_cmd_np = (1.0 - alpha) * commanded_np + alpha * measured_np

        cmd = MotionCommand()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.control_mode = status.active_control_mode
        cmd.joint_position = list_to_jvq(filtered_cmd_np)
        self.side.motion_command.publish(cmd)


class ManualMotion(Node):

    def __init__(self):
        super().__init__("victor_manual_motion")
        self.victor = Victor(self)
        self.create_timer(0.1, self.update)

        self.left_filter = ManualMotionFilter(self, self.victor.left)
        self.right_filter = ManualMotionFilter(self, self.victor.right)

    def update(self):
        self.left_filter.update()
        self.right_filter.update()


def main():
    rclpy.init()
    mm = ManualMotion()
    rclpy.spin(mm)


if __name__ == '__main__':
    main()
