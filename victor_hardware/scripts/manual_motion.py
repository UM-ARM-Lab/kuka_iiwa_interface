import rclpy
from victor_hardware.victor import Victor
from rclpy.node import Node

class ManualMotion(Node):

    def __init__(self):
        super().__init__("victor_manual_motion")

        self.victor = Victor(self)

def main():
    rclpy.init()

    mm = ManualMotion()

    rclpy.spin()

if __name__ == '__main__':
    main()