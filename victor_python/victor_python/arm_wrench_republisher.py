#!/usr/bin/env python3
"""
Listens to robot feedback, extracts and republishes just the wrench data
Useful for plotting directly in rviz, or otherwise.
"""


import rclpy
from victor_hardware_interfaces.msg import MotionStatus
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3

# This frame has to match the frame in the URDF,
# which must match the frame used in the Sunrise code when computing external wrench
hand_to_frame = {'left': 'victor_left_arm_sunrise_palm_surface',
                 'right': 'victor_right_arm_sunrise_palm_surface'}


def publish_wrench(data, hand, hand_wrench_pub):
    wr = data.estimated_external_wrench
    wrench_msg = Wrench()
    wrench_msg.force = Vector3(x=wr.x, y=wr.y, z=wr.z)
    wrench_msg.torque = Vector3(x=wr.c, y=wr.b, z=wr.a)

    wrench_stamped_msg = WrenchStamped()
    wrench_stamped_msg.wrench = wrench_msg
    wrench_stamped_msg.header.stamp = data.header.stamp  # Use the timestamp from the incoming data
    wrench_stamped_msg.header.frame_id = hand_to_frame[hand]

    hand_wrench_pub[hand].publish(wrench_stamped_msg)


def main():
    rclpy.init()
    node = rclpy.create_node('wrench_republisher')
    node.get_logger().info('Wrench republisher started')

    hand_wrench_pub = {
        'left': node.create_publisher(WrenchStamped, 'left_gripper/wrench', 10),
        'right': node.create_publisher(WrenchStamped, 'right_gripper/wrench', 10)
    }

    left_subscriber = node.create_subscription(MotionStatus, 'left_arm/motion_status',
                                               lambda msg: publish_wrench(msg, 'left', hand_wrench_pub), 10)
    right_subscriber = node.create_subscription(MotionStatus, 'right_arm/motion_status',
                                                lambda msg: publish_wrench(msg, 'right', hand_wrench_pub), 10)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
