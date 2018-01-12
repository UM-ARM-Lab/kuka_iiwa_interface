import rospy

from victor_hardware_interface.msg import *
from victor_hardware_interface.srv import *

class MinimalFakeArmInterface:
    def __init__(self,
                 motion_command_topic,
                 motion_status_topic,
                 control_mode_status_topic,
                 get_control_mode_service_topic,
                 set_control_mode_service_topic,
                 gripper_command_topic,
                 gripper_status_topic):

        self.control_status_pub =   rospy.Publisher(control_mode_status_topic,          ControlModeParameters,  queue_size=1)
        self.motion_status_pub =    rospy.Publisher(motion_status_topic,                MotionStatus,           queue_size=1)
        self.gripper_status_pub =   rospy.Publisher(gripper_status_topic,               Robotiq3FingerStatus,   queue_size=1)

        self.motion_command_sub =   rospy.Subscriber(motion_command_topic,              MotionCommand,          self.arm_motion_command_callback)
        self.gripper_command_sub =  rospy.Subscriber(gripper_command_topic,             Robotiq3FingerCommand,  self.gripper_command_callback)

        self.get_control_mode_server = rospy.Service(set_control_mode_service_topic,    GetControlMode,         self.get_control_mode_service_callback)
        self.set_control_mode_server = rospy.Service(set_control_mode_service_topic,    SetControlMode,         self.set_control_mode_service_callback)

        pass

    def arm_motion_command_callback(self, cmd):
        pass

    def gripper_command_callback(self, cmd):
        pass

    def get_control_mode_service_callback(self, req):
        pass

    def set_control_mode_service_callback(self, req):
        pass


if __name__ == "__main__":
    rospy.init_node("minimal_fake_arm_interface")