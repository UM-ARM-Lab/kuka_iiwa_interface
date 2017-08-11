
import rospkg
import rospy

import math
import sys
import openravepy as rave

class victor_openrave:
    def __init__(self):
        self.initialize_openrave()
        self.load_robot()


    def initialize_openrave(self):
        # First create an environment so that we can load plugins
        rave.misc.InitOpenRAVELogging()
        self.env = rave.Environment()
        self.env.SetCollisionChecker(rave.RaveCreateCollisionChecker(self.env, 'ode'))
        # attach viewer
        self.env.SetViewer('qtcoin')
        self.env.GetViewer().SetCamera([
            [0., 0., 1., -3.6],
            [-1., 0., 0., 0.],
            [0., -1., 0., 1.5],
            [0., 0., 0., 1.]
        ])
        self.env.GetViewer().SetSize(800, 1028)

        # Load the environment data
        # http://wiki.ros.org/Packages#Python
        # rospack = rospkg.RosPack()
        # path = rospack.get_path('mps_launch_files')
        # self.env.Load(path + '/config/armlab_setup.env.xml')
        # # Set the transform in OpenRAVE
        # with self.env:
        #     table = self.env.GetKinBody('Table')
        #     table.SetTransform(self.table_transform)

        #     tray = self.env.GetKinBody('Tray')
        #     tray_transform = self.table_surface_transform
        #     tray_transform[1,3] += -0.3325
        #     tray_transform[2,3] += 0.017
        #     tray.SetTransform(tray_transform)

        #     basket = self.env.GetKinBody('Basket')
        #     basket_transform = basket.GetTransform()
        #     basket_transform[0:2, 3] = self.deposit_transform[0:2, 3]
        #     basket_transform[2, 3] = self.table_surface_transform[2, 3] + 0.03
        #     basket.SetTransform(basket_transform)

        #     self.move_fake_obstacle_away()

    def load_robot(self):
        # Setup the C++ plugin ROS code
        # plugin_argv = ros_argv_conversion.convert_argv_to_string(sys.argv, '_internal_plugin')
        # ros_initializer = rave.RaveCreateModule(self.env, 'orrosplugininitializer')
        # ros_initializer.SendCommand('Initialize ' + plugin_argv)

        # Load the robot itself
        urdf_module = rave.RaveCreateModule(self.env, 'urdf')
        with self.env:
            name = urdf_module.SendCommand(
                'LoadURI package://victor_description/urdf/victor.urdf package://victor_moveit_config/config/victor.srdf')
            self.robot = self.env.GetRobot(name)
        # Use the ROS based controller - listens to joint positions and sets joint commands
        # controller = rave.RaveCreateController(self.env, 'VictorROSJointPositionController ignored_args')
        # self.robot.SetController(controller, range(self.robot.GetDOF()), controltransform = 0)
        # with self.env:
        #     self.robot.SetTransform(self.robot_base_transform)

        # Define which manipulator we are using
        self.robot.SetActiveManipulator('right_arm')
        self.manip = self.robot.GetActiveManipulator()
        self.robot.SetActiveDOFs(self.manip.GetArmIndices())

        # Create an ik model to be used by move_object_callback
        # self.ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(self.robot, iktype = rave.IkParameterizationType.Transform6D)
        # if not self.ikmodel.load():
        #     self.ikmodel.autogenerate()

        self.victor_right_arm_joint_names = [
            'victor_right_arm_joint_1',
            'victor_right_arm_joint_2',
            'victor_right_arm_joint_3',
            'victor_right_arm_joint_4',
            'victor_right_arm_joint_5',
            'victor_right_arm_joint_6',
            'victor_right_arm_joint_7']
        self.victor_right_arm_joint_indices = [self.robot.GetJoint(name).GetDOFIndex() for name in self.victor_right_arm_joint_names]
