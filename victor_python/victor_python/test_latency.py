#!/usr/bin/env python3
# Subscribe to the kinect points and the MotionStatus and compute the latency between the time stamp in the messsage
# and the time the message was received.
# Then plot this over time for 30 seconds of data.
import socket
from time import time

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import matplotlib.pyplot as plt

from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from victor_hardware_interfaces.msg import MotionStatus


class LatencyTester(Node):
    def __init__(self, msg_type, topic):
        self.topic_underscores = topic.replace('/', '_')
        super().__init__(f'{self.topic_underscores}_latency_tester')
        self.sub = self.create_subscription(msg_type, topic, self.cb, 10)
        self.latency_ns = []

    def cb(self, msg):
        latency: Duration = self.get_clock().now() - Time.from_msg(msg.header.stamp)
        self.latency_ns.append(latency.nanoseconds)


def main():
    rclpy.init()
    pc = LatencyTester(PointCloud2, '/k4a/points')
    motion_status = LatencyTester(MotionStatus, '/victor/left_arm/motion_status')
    nodes = [
        pc,
        motion_status,
    ]

    t0 = time()
    while (time() - t0) < 30:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.1)

    rclpy.shutdown()

    hostname = socket.gethostname()

    for n in nodes:
        plt.figure()
        plt.title(f'Latency over time [{n.sub.topic_name}]')
        plt.xlabel('Message number')
        plt.ylabel('Latency (ms)')
        plt.plot(np.array(n.latency_ns) / 1e6)
        plt.savefig(f'latency_{hostname}_{n.topic_underscores}.png')

    plt.show()


if __name__ == '__main__':
    main()
