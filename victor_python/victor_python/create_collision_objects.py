"""
Moves the final joint of the right arm by +/- 0.5 radians.
"""
from copy import copy

import numpy as np
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject

from arm_robots.robot import load_moveitpy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from threading import Thread

ROBOT_NAME = "victor"

# This has to match a group in the SRDF file describing the robot.
GROUP_NAME = "right_arm"

JOINT_7_DELTA_DEG = 30


def main():
    rclpy.init()
    node = Node('test_get_controller')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin)
    spin_thread.start()
    node.create_rate(1.0).sleep()
    print("Started spin thread")
    collision_object_publisher = node.create_publisher(CollisionObject, '/collision_object', 10)
    print("Publisher created")
    object_positions = [
        (0.15, 0.1, 0.5),
        (0.25, 0.0, 1.0),
        (-0.25, -0.3, 0.8),
        (0.25, 0.3, 0.75),
    ]
    object_dimensions = [
        (0.1, 0.4, 0.1),
        (0.1, 0.4, 0.1),
        (0.2, 0.2, 0.2),
        (0.15, 0.15, 0.15),
    ]

    collision_object = CollisionObject()
    collision_object.header.frame_id = "/victor_root"
    collision_object.header.stamp = node.get_clock().now().to_msg()
    collision_object.id = "boxes"

    for position, dimensions in zip(object_positions, object_dimensions):
        box_pose = Pose()
        box_pose.position.x = position[0]
        box_pose.position.y = position[1]
        box_pose.position.z = position[2]

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
    print("Publishing collision object")
    collision_object_publisher.publish(collision_object)

if __name__ == "__main__":
    main()
