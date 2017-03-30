#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
#include <wiktor_hardware_interface/iiwa_hardware_interface.hpp>
#include <wiktor_hardware_interface/robotiq_3finger_hardware_interface.hpp>
// ROS message headers
#include <wiktor_hardware_interface/ControlModeCommand.h>
#include <wiktor_hardware_interface/ControlModeStatus.h>
#include <wiktor_hardware_interface/MotionCommand.h>
#include <wiktor_hardware_interface/MotionStatus.h>
#include <wiktor_hardware_interface/Robotiq3FingerCommand.h>
#include <wiktor_hardware_interface/Robotiq3FingerStatus.h>
// ROS
#include <ros/ros.h>
// LCM
#include <lcm/lcm-cpp.hpp>

class LoopbackTester
{
protected:

    ros::NodeHandle nh_;
    std::shared_ptr<lcm::LCM> lcm_ptr_;
    std::unique_ptr<iiwa_hardware_interface::IIWAHardwareInterface> iiwa_ptr_;
    std::unique_ptr<robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface> robotiq_ptr_;

public:

    LoopbackTester(ros::NodeHandle& nh, const std::shared_ptr<lcm::LCM>& lcm_ptr, const std::string& motion_command_channel, const std::string& motion_status_channel, const std::string& control_mode_command_channel, const std::string& control_mode_status_channel, const std::string& gripper_command_channel, const std::string& gripper_status_channel) : nh_(nh), lcm_ptr_(lcm_ptr)
    {
        std::function<void(const wiktor_hardware_interface::MotionStatus&)> motion_status_callback_fn = [&] (const wiktor_hardware_interface::MotionStatus& motion_status) { return MotionStatusCallback(motion_status); };
        std::function<void(const wiktor_hardware_interface::ControlModeStatus&)> control_mode_status_callback_fn = [&] (const wiktor_hardware_interface::ControlModeStatus& control_mode_status) { return ControlModeStatusCallback(control_mode_status); };
        iiwa_ptr_ = std::unique_ptr<iiwa_hardware_interface::IIWAHardwareInterface>(new iiwa_hardware_interface::IIWAHardwareInterface(lcm_ptr_, motion_command_channel, motion_status_channel, motion_status_callback_fn, control_mode_command_channel, control_mode_status_channel, control_mode_status_callback_fn));
        std::function<void(const wiktor_hardware_interface::Robotiq3FingerStatus&)> gripper_status_callback_fn = [&] (const wiktor_hardware_interface::Robotiq3FingerStatus& gripper_status) { return GripperStatusCallback(gripper_status); };
        robotiq_ptr_ = std::unique_ptr<robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface>(new robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface(lcm_ptr_, gripper_command_channel, gripper_status_channel, gripper_status_callback_fn));
    }

    void MotionStatusCallback(const wiktor_hardware_interface::MotionStatus& motion_status)
    {
        ROS_INFO_STREAM_NAMED(ros::this_node::getName(), "Got motion status " << motion_status);
    }

    void ControlModeStatusCallback(const wiktor_hardware_interface::ControlModeStatus& control_mode_status)
    {
        ROS_INFO_STREAM_NAMED(ros::this_node::getName(), "Got control mode status " << control_mode_status);
    }

    void GripperStatusCallback(const wiktor_hardware_interface::Robotiq3FingerStatus& gripper_status)
    {
        ROS_INFO_STREAM_NAMED(ros::this_node::getName(), "Got gripper status " << gripper_status);
    }

    void Loop()
    {
        ;
    }
};

int main(int argc, char** argv)
{
    // Default parameter values
    const std::string DEFAULT_LCM_URL("");
    const std::string DEFAULT_MOTION_COMMAND_CHANNEL("motion_command");
    const std::string DEFAULT_MOTION_STATUS_CHANNEL("motion_status");
    const std::string DEFAULT_CONTROL_MODE_COMMAND_CHANNEL("control_mode_command");
    const std::string DEFAULT_CONTROL_MODE_STATUS_CHANNEL("control_mode_status");
    const std::string DEFAULT_GRIPPER_COMMAND_CHANNEL("gripper_command");
    const std::string DEFAULT_GRIPPER_STATUS_CHANNEL("gripper_status");
    // Start ROS
    ros::init(argc, argv, "loopback_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string lcm_url = nhp.param(std::string("lcm_url"), DEFAULT_LCM_URL);
    const std::string motion_command_channel = nhp.param(std::string("motion_command_channel"), DEFAULT_MOTION_COMMAND_CHANNEL);
    const std::string motion_status_channel = nhp.param(std::string("motion_status_channel"), DEFAULT_MOTION_STATUS_CHANNEL);
    const std::string control_mode_command_channel = nhp.param(std::string("control_mode_command_channel"), DEFAULT_CONTROL_MODE_COMMAND_CHANNEL);
    const std::string control_mode_status_channel = nhp.param(std::string("control_mode_status_channel"), DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
    const std::string gripper_command_channel = nhp.param(std::string("gripper_command_channel"), DEFAULT_GRIPPER_COMMAND_CHANNEL);
    const std::string gripper_status_channel = nhp.param(std::string("gripper_status_channel"), DEFAULT_GRIPPER_STATUS_CHANNEL);
    // Start LCM
    std::shared_ptr<lcm::LCM> lcm_ptr(new lcm::LCM(lcm_url));
    ROS_INFO("Starting Loopback Test Node...");
    LoopbackTester tester(nh, lcm_ptr, motion_command_channel, motion_status_channel, control_mode_command_channel, control_mode_status_channel, gripper_command_channel, gripper_status_channel);
    tester.Loop();
    return 0;
}
