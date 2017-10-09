#! /usr/bin/env python

import math
import time
import numpy as np
from sklearn import preprocessing
from scipy.spatial import ConvexHull
import IPython
from copy import deepcopy
from threading import Lock

import openravepy as rave

import rospkg
import rospy
import actionlib

import tf.transformations
import tf2_ros
import tf2_geometry_msgs

from visualization_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from victor_hardware_interface.msg import *
from victor_hardware_interface.srv import *
from mps_msgs.msg import *


from arc_utilities import transformation_helper
from arc_utilities import numpy_conversions
from or_ros_plugin_initializer import ros_argv_conversion


ARM_NAMES = ["right_arm", "left_arm"]

class Planner:
    def __init__(self):
        print "Initializing Planner"
        self.initialize_transforms()
        self.initialize_openrave()
        self.load_robot()
        # self.initialize_control_mode_service_client()
        self.initialize_arm_motion_pubsub()
        self.initialize_gripper_messages()
        


        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
        # self.perception_srv_client = rospy.ServiceProxy("beanBagLocateService", GetPerception)


        # self.test_ik_solns()

    def initialize_transforms(self):
        # Initialize all the static transforms we will need
        self.world_frame = "victor_root"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.robot_base_transform = self.get_tf_transform(self.world_frame, "victor_root")
        self.palm_surface_to_fingertips_dist = 0.10667743
        # self.table_surface_transform = self.get_tf_transform(self.world_frame, "table_surface")
        # self.deposit_transform = self.get_tf_transform(self.world_frame, "deposit_location")
        # self.manip_ee_to_palm_surface_kuka = self.get_tf_transform("victor_right_gripper_openrave_manipulator_ee", "victor_right_gripper_palm_surface_kuka")
        # self.right_arm_kuka_world_frame = self.get_tf_transform(self.world_frame, "victor_right_arm_world_frame_kuka")

    def initialize_openrave(self):
        """Creates the openrave environment"""
        # First create an environment so that we can load plugins
        rave.misc.InitOpenRAVELogging()
        self.env = rave.Environment()
        self.env.SetCollisionChecker(rave.RaveCreateCollisionChecker(self.env, "ode"))
        # attach viewer
        self.env.SetViewer("qtcoin")
        self.env.GetViewer().SetCamera([
            [0., 0., 1., -3.6],
            [-1., 0., 0., 0.],
            [0., -1., 0., 1.5],
            [0., 0., 0., 1.]
        ])
        self.env.GetViewer().SetSize(800, 1028)

        # Load the environment data
        # http://wiki.ros.org/Packages#Python
        rospack = rospkg.RosPack()
        path = rospack.get_path("victor_motion_planner")
        self.env.Load(path + "/config/armlab_setup.env.xml")
        # Set the transform in OpenRAVE

    def set_manipulator(self, manipulator_name):
        """Sets the active manipulator and updates the active DOFs
        current, 
        manipulator_name: either "left_arm" or "right_arm"
        """
        try:
            self.robot.SetActiveManipulator(manipulator_name)
        except rave.openrave_exception as err:
            print "\nINVALID MANIPULATOR NAME\n"
            raise err
            
        self.manip = self.robot.GetActiveManipulator()
        self.robot.SetActiveDOFs(self.manip.GetArmIndices())
        self.manipulator_name = manipulator_name


    def load_robot(self):
        """
        loads the openrave configuration files, builds the robot controller, and load (or generates) ik solutions for the manipulator
        """
        
        # Setup the C++ plugin ROS code
        plugin_argv = ros_argv_conversion.convert_argv_to_string(sys.argv, "_internal_plugin")
        ros_initializer = rave.RaveCreateModule(self.env, "orrosplugininitializer")
        ros_initializer.SendCommand("Initialize " + plugin_argv)

        # Load the robot itself
        urdf_module = rave.RaveCreateModule(self.env, "urdf")
        with self.env:
            victor_urdf_filename = "package://victor_description/urdf/victor.urdf"
            victor_srdf_filename = "package://victor_moveit_config/config/victor.srdf"
            name = urdf_module.SendCommand("LoadURI " + victor_urdf_filename + " " + victor_srdf_filename)
            self.robot = self.env.GetRobot(name)
        # Use the ROS based controller - listens to joint positions and sets joint commands
        controller = rave.RaveCreateController(self.env, "VictorROSJointPositionController ignored_args")
        self.robot.SetController(controller, range(self.robot.GetDOF()), controltransform = 0)
        with self.env:
            self.robot.SetTransform(self.robot_base_transform)

        # Define which manipulator we are using
        # self.robot.SetActiveManipulator("right_arm")

        for manipulator_name in ARM_NAMES:
            self.set_manipulator(manipulator_name)

            # Create an ik model to be used by move_object_callback
            ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype = rave.IkParameterizationType.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()

        self.victor_right_arm_joint_names = ["victor_right_arm_joint_" + str(i) for i in range(1,7)]
        self.victor_left_arm_joint_names = ["victor_left_arm_joint_" + str(i) for i in range(1,7)]
        self.victor_right_arm_joint_indices = [self.robot.GetJoint(name).GetDOFIndex() for name in self.victor_right_arm_joint_names]
        self.victor_left_arm_joint_indices = [self.robot.GetJoint(name).GetDOFIndex() for name in self.victor_left_arm_joint_names]

        # print "left arm joint[0] index: " + str(self.victor_left_arm_joint_indices[0])
        # print "right arm joint[0] index: " + str(self.victor_right_arm_joint_indices[0])

        print "Robot loaded"
        
    # def initialize_control_mode_service_client(self):
    #     self.set_control_mode_client = rospy.ServiceProxy("/right_arm/set_control_mode_service", SetControlMode)

    def default_gripper_command(self):
        cmd = Robotiq3FingerCommand()
        cmd.finger_a_command.speed = 0.5
        cmd.finger_b_command.speed = 0.5
        cmd.finger_c_command.speed = 0.5
        cmd.scissor_command.speed = 1.0

        cmd.finger_a_command.force = 1.0
        cmd.finger_b_command.force = 1.0
        cmd.finger_c_command.force = 1.0
        cmd.scissor_command.force = 1.0

        cmd.scissor_command.position = 1.0
        return cmd


    def initialize_gripper_messages(self):
        self.right_gripper_command = self.default_gripper_command()
        self.left_gripper_command = self.default_gripper_command()
        

        self.gripper_input_lock = Lock()
        self.right_gripper_status = Robotiq3FingerStatus()
        self.left_gripper_status = Robotiq3FingerStatus()

        self.right_gripper_status_subscriber = rospy.Subscriber("right_arm/gripper_status", Robotiq3FingerStatus, self.right_gripper_status_callback)
        self.right_gripper_command_publisher = rospy.Publisher("right_arm/gripper_command", Robotiq3FingerCommand, queue_size = 1)

        self.left_gripper_status_subscriber = rospy.Subscriber("left_arm/gripper_status", Robotiq3FingerStatus, self.left_gripper_status_callback)
        self.left_gripper_command_publisher = rospy.Publisher("left_arm/gripper_command", Robotiq3FingerCommand, queue_size = 1)

        while(self.right_gripper_command_publisher.get_num_connections() == 0 or
              self.left_gripper_command_publisher.get_num_connections() == 0):
            rospy.timer.Rate(100).sleep()
            print "waiting for connection"


    def initialize_arm_motion_pubsub(self):
        self.motion_status_input_lock = Lock()
        self.right_arm_motion_status = MotionStatus()
        self.left_arm_motion_status = MotionStatus()

        self.right_arm_motion_status_subscriber = rospy.Subscriber("right_arm/motion_status", MotionStatus, self.right_arm_motion_status_callback)
        self.right_arm_motion_command_publisher = rospy.Publisher("right_arm/motion_command", MotionCommand, queue_size = 1)
        self.left_arm_motion_status_subscriber = rospy.Subscriber("left_arm/motion_status", MotionStatus, self.left_arm_motion_status_callback)
        self.left_arm_motion_command_publisher = rospy.Publisher("left_arm/motion_command", MotionCommand, queue_size = 1)



    def get_tf_transform(self, parent, child, verbose=False):
        try:
            while not rospy.is_shutdown() and not self.tf_buffer.can_transform(child, parent, rospy.Time(), rospy.Duration(secs=0, nsecs=500*1000*1000)):
                print "Waiting for TF frames ", parent, " and ", child
            transform = self.tf_buffer.lookup_transform(parent, child, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No transform available: %s to %s", parent, child)
            return

        return transformation_helper.BuildMatrixRos(transform.transform.translation, transform.transform.rotation)

    def send_tf_transform(self, transform, parent, child):
        [translation, quaternion] = transformation_helper.ExtractFromMatrix(transform)
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(t)


    def move_hand_straight(self, moving_direction, moving_distance, step_size=0.005, execute=False, waitrobot=False, disable_table_collision=True):
        """
        Wrapper around openrave's MoveHandStraight

        moving_direction: [x,y,z] vector in world frame. 
        moving_distance: moving distance in meters
        step_size: IK interpolation step size in meters
        execute: if True, will execute the path using the controller
        wait_robot: if True will sleep until the robot controller to finish trajectory
        """
        print "Planning " + self.manipulator_name + " to target"
        
        manip_problem = rave.interfaces.BaseManipulation(self.robot)
        try:
            traj = manip_problem.MoveHandStraight(direction=moving_direction,
                                                  stepsize=step_size,
                                                  execute=execute,
                                                  outputtraj=True,
                                                  outputtrajobj=True,
                                                  ignorefirstcollision=True,
                                                  minsteps=1,
                                                  maxsteps=int(round(moving_distance / step_size)))
        except rave.PlanningError as ex:
            print ex
            IPython.embed()
            traj = None

        if execute and waitrobot:
            self.wait_robot()

        return traj


    def plan_to_configuration(self, target_config, execute=False, use_fake_obstacle=False):
        """
        Plans and optionally executes a path for the active manipulator to the specified target
        
        target_config: list of joint angles for the target_config
        execute: if True, will execute the planned trajectory using the robot's controller
        use_fake: depricated
        """

        print "Planning " + self.manipulator_name + " to target configuration:"
        print np.array(target_config)*180/np.pi
        if use_fake_obstacle:
            self.move_fake_obstacle_above_table()

        manip_problem = rave.interfaces.BaseManipulation(self.robot)
        arm_traj = manip_problem.MoveManipulator(goal = target_config, execute=False, outputtrajobj=True)
        rave.planningutils.SmoothActiveDOFTrajectory(arm_traj, self.robot)

        if use_fake_obstacle:
            self.move_fake_obstacle_away()
            

        if execute:

            data = []
            print "Moving to target"

            with self.env:
                # print arm_traj

                # for i in range(arm_traj.GetNumWaypoints()):
                #     print np.array(arm_traj.GetWaypoint(i))*180/np.pi

                self.robot.GetController().SetPath(arm_traj)
            self.wait_robot()

        return arm_traj

    def plan_to_relative_pose(self, relative_pose, execute=False):
        """
        plans and optionally executes the active manipulator to a pose relative to the current pose

        relative_pose: The relative pose as a 4x4 matrix
        execute: if True, will execute the planned trajectory using the robot's controller
        """
        current_pose = self.manip.GetEndEffectorTransform()
        return(self.plan_to_pose(current_pose.dot(relative_pose), execute))

    def plan_to_pose(self, target_pose, execute=False):
        """
        Plans and optionally executes the active manipulator to the target_pose
        
        target_pose: The target pose for the planner as a 4x4 matrix
        execute: if True, will execute the planned trajectory using the robot's controller
        """

        target_config = self.manip.FindIKSolution(target_pose, rave.IkFilterOptions.CheckEnvCollisions)

        if target_config is None:
            print "   --------   Plan to pose: unable to get IK solution"
            return None
        return self.plan_to_configuration(target_config, execute = execute)


    def get_close_ik(self, target_pose, allow_rotation_change=False, min_z_val=None):
        with self.env:
            i = 1
            while True:
                print("Checking solution " + str(i))
                solution = self.manip.FindIKSolution(target_pose, 0)
                if solution is not None:
                    break
                target_pose[0:3, 3] = target_pose[0:3, 3] + (np.random.rand(3) - 0.5) * 0.001
                if min_z_val is not None:
                    target_pose[2, 3] = max(min_z_val, target_pose[2, 3])
                if allow_rotation_change:
                    target_pose[0:3, 0:3] = np.dot(target_pose[0:3, 0:3], rave.rotationMatrixFromAxisAngle((np.random.rand(3) - 0.5) * 0.001))
                self.send_tf_transform(target_pose, self.world_frame, "random_target")
                i += 1
            return solution

    def wait_robot(self):
        """busy wait for robot completion"""
        while not self.robot.GetController().IsDone():
            time.sleep(0.01)


    def right_arm_motion_status_callback(self, status):
        with self.motion_status_input_lock:
            self.right_arm_motion_status = status

    def left_arm_motion_status_callback(self, status):
        with self.motion_status_input_lock:
            self.left_arm_motion_status = status


    def right_gripper_status_callback(self, status):
        with self.gripper_input_lock:
            self.right_gripper_status = status

    def left_gripper_status_callback(self, status):
        with self.gripper_input_lock:
            self.left_gripper_status = status

    def wait_gripper(self):
        """Waits for the gripper to close
        TODO - actually use gripper feedback, dont hard code a time"""
        time.sleep(3)

    def close_right_gripper(self, blocking=True):
        self.right_gripper_command.finger_a_command.position = 1
        self.right_gripper_command.finger_b_command.position = 1
        self.right_gripper_command.finger_c_command.position = 1
        self.right_gripper_command_publisher.publish(self.right_gripper_command)

        if blocking:
            self.wait_gripper()

    def open_right_gripper(self, blocking=True):
        self.right_gripper_command.finger_a_command.position = 0
        self.right_gripper_command.finger_b_command.position = 0
        self.right_gripper_command.finger_c_command.position = 0
        self.right_gripper_command_publisher.publish(self.right_gripper_command)

        if blocking:
            self.wait_gripper()

    def close_left_gripper(self, blocking=True):
        self.left_gripper_command.finger_a_command.position = 1
        self.left_gripper_command.finger_b_command.position = 1
        self.left_gripper_command.finger_c_command.position = 1
        self.left_gripper_command_publisher.publish(self.left_gripper_command)

        if blocking:
            self.wait_gripper()

    def open_left_gripper(self, blocking=True):
        self.left_gripper_command.finger_a_command.position = 0
        self.left_gripper_command.finger_b_command.position = 0
        self.left_gripper_command.finger_c_command.position = 0
        self.left_gripper_command_publisher.publish(self.left_gripper_command)

        if blocking:
            self.wait_gripper()


    def move_object_callback(self, move_object_goal):
        feedback = MoveObjectFeedback()
        result = MoveObjectResult()

        try:
            print "Determining pickup position"
            feedback.stage = MoveObjectFeedback.MOVING_TO_GRAB
            self.move_object_as.publish_feedback(feedback)
            beanbag_transform, region_half_side_length = self.get_beanbag_transform_and_size(move_object_goal.header.frame_id, move_object_goal.target_region.points)
            manipulator_approach_transform, manipulator_target_transform = self.get_manipulator_target_above_target_object(beanbag_transform, region_half_side_length)

            # print "Waiting for confirmation before moving"
            # IPython.embed()

            print "Planning and moving to above pickup position"
            arm_trajectory = self.plan_to_pose(manipulator_approach_transform, execute=True, use_fake_obstacle=True)
            if arm_trajectory is None:
                print "   --------   Unable to pickup beanbag, doing nothing"
                # self.push_region(manipulator_target_transform_above_beanbag)
            else:
                print "Moving down to contact bean bag"
                self.move_down_to_grasp_pose(manipulator_target_transform)

                print "Closing gripper"
                feedback.stage = MoveObjectFeedback.GRABBING
                self.move_object_as.publish_feedback(feedback)
                self.close_gripper()

                # print "Waiting for confirmation before raising gripper"
                # IPython.embed()

                print "Clearing visualization markers"
                self.delete_target_object_green_box()
                self.delete_approach_vector_marker()

                print "Moving up from grasp pose"
                self.move_up_from_grasp_pose(manipulator_approach_transform)
                
                print "Move up to verification pose"
                # self.move_to_verification_pose

                print "Planning and moving to release location"
                feedback.stage = MoveObjectFeedback.MOVING_TO_RELEASE
                self.move_object_as.publish_feedback(feedback)
                if move_object_goal.action_type == MoveObjectGoal.PUSH:
                    print "Asked to push, but I can\"t push, so discarding object"
                    bean_bag_release_config = self.discard_configuration
                elif move_object_goal.action_type == MoveObjectGoal.DISCARD:
                    print "Asked to discard"
                    bean_bag_release_config = self.discard_configuration
                elif move_object_goal.action_type == MoveObjectGoal.RETRIEVE:
                    print "Asked to retrieve"
                    bean_bag_release_config = self.retrieve_configuration
                else:
                    print "Invalid action type"
                    result.success = False
                    self.move_object_as.set_aborted(result)
                    return
                self.plan_to_configuration(bean_bag_release_config, execute=True, use_fake_obstacle=True)

                print "Opening gripper"
                feedback.stage = MoveObjectFeedback.RELEASING
                self.move_object_as.publish_feedback(feedback)
                self.open_gripper()

            print "Moving home"
            feedback.stage = MoveObjectFeedback.MOVING_TO_HOME
            self.move_object_as.publish_feedback(feedback)
            self.plan_to_configuration(self.home_configuration, execute=True, use_fake_obstacle=True)

        finally:
            self.delete_target_object_green_box()
            self.delete_approach_vector_marker()

        result.success = True
        self.move_object_as.set_succeeded(result)

        print "Ready for next target"


    def test_ik_solns(self):

        marker_fingers = Marker()
        marker_fingers.header.frame_id = "victor_root"
        marker_fingers.type = Marker.POINTS
        marker_fingers.ns = "finger_reachability_vis"
        marker_fingers.id = 1
        marker_fingers.action = Marker.ADD
        marker_fingers.scale.x = 0.01
        marker_fingers.scale.y = 0.01
        marker_fingers.scale.z = 0.01
        marker_fingers.color.r = 1.0
        marker_fingers.color.g = 0.0
        marker_fingers.color.b = 0.0
        marker_fingers.color.a = 1.0

        target_pose = transformations.quaternion_matrix([1.0, 0.0, 0.0, 0.0])
        for x in np.linspace(1.5, 2.4, 20):
            target_pose[0, 3] = x
            for y in np.linspace(-0.9, 1.2, 20):
                target_pose[1, 3] = y
                for z in np.linspace(0.84, 1.30, 20):
                    target_pose[2, 3] = z
                    target_config = self.manip.FindIKSolution(target_pose, rave.IkFilterOptions.CheckEnvCollisions)
                    if target_config is not None:
                        finger_target = target_pose.dot(self.manip_ee_to_palm_surface_transform)
                        finger_target[2, 3] -= self.palm_surface_to_fingertips_dist

                        finger_target_in_victor_frame = np.linalg.inv(self.robot_base_transform).dot(finger_target)

                        p_fingers = Point()
                        p_fingers.x = finger_target_in_victor_frame[0, 3]
                        p_fingers.y = finger_target_in_victor_frame[1, 3]
                        p_fingers.z = finger_target_in_victor_frame[2, 3]
                        marker_fingers.points.append(p_fingers)

        marker_fingers.header.stamp = rospy.Time.now()
        self.marker_pub.publish(marker_fingers)

        # IPython.embed()


if __name__ == "__main__":
    rospy.init_node("openrave_planner")
    planner = Planner()
    planner.start()
    rospy.spin()
