#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <mutex>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/maybe.hpp>
#include <victor_hardware_interface/iiwa_hardware_interface.hpp>
#include <victor_hardware_interface/robotiq_3finger_hardware_interface.hpp>
#include <victor_hardware_interface/minimal_arm_wrapper_interface.hpp>
// ROS message headers
#include <victor_hardware_interface/ControlModeCommand.h>
#include <victor_hardware_interface/ControlModeStatus.h>
#include <victor_hardware_interface/MotionCommand.h>
#include <victor_hardware_interface/MotionStatus.h>
#include <victor_hardware_interface/Robotiq3FingerCommand.h>
#include <victor_hardware_interface/Robotiq3FingerStatus.h>
#include <victor_hardware_interface/SetControlMode.h>
#include <victor_hardware_interface/GetControlMode.h>
// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
// LCM
#include <lcm/lcm-cpp.hpp>

int main(int argc, char** argv)
{
    // Default ROS params
    const std::string DEFAULT_CARTESIAN_POSE_FRAME = "base";
    // Default ROS topic / service names
    const std::string DEFAULT_MOTION_COMMAND_TOPIC("motion_command");
    const std::string DEFAULT_MOTION_STATUS_TOPIC("motion_status");
    const std::string DEFAULT_CONTROL_MODE_STATUS_TOPIC("control_mode_status");
    const std::string DEFAULT_SET_CONTROL_MODE_SERVICE("set_control_mode_service");
    const std::string DEFAULT_GET_CONTROL_MODE_SERVICE("get_control_mode_service");
    const std::string DEFAULT_GRIPPER_COMMAND_TOPIC("gripper_command");
    const std::string DEFAULT_GRIPPER_STATUS_TOPIC("gripper_status");
    // Default LCM parameters
    const std::string DEFAULT_SEND_LCM_URL("udp://10.10.10.11:30000");
    const std::string DEFAULT_RECV_LCM_URL("udp://10.10.10.100:30001");
    const std::string DEFAULT_MOTION_COMMAND_CHANNEL("motion_command");
    const std::string DEFAULT_MOTION_STATUS_CHANNEL("motion_status");
    const std::string DEFAULT_CONTROL_MODE_COMMAND_CHANNEL("control_mode_command");
    const std::string DEFAULT_CONTROL_MODE_STATUS_CHANNEL("control_mode_status");
    const std::string DEFAULT_GRIPPER_COMMAND_CHANNEL("gripper_command");
    const std::string DEFAULT_GRIPPER_STATUS_CHANNEL("gripper_status");
    const double DEFAULT_SET_CONTROL_MODE_TIMEOUT = 2.5;
    // Start ROS
    ros::init(argc, argv, "right_arm_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    // Get params
    const std::string cartesian_pose_frame = nhp.param(std::string("cartesian_pose_frame"), DEFAULT_CARTESIAN_POSE_FRAME);
    // Get topic & service names
    const std::string motion_command_topic = nhp.param(std::string("motion_command_topic"), DEFAULT_MOTION_COMMAND_TOPIC);
    const std::string motion_status_topic = nhp.param(std::string("motion_status_topic"), DEFAULT_MOTION_STATUS_TOPIC);
    const std::string control_mode_status_topic = nhp.param(std::string("control_mode_status_topic"), DEFAULT_CONTROL_MODE_STATUS_TOPIC);
    const std::string get_control_mode_service = nhp.param(std::string("get_control_mode_service"), DEFAULT_GET_CONTROL_MODE_SERVICE);
    const std::string set_control_mode_service = nhp.param(std::string("set_control_mode_service"), DEFAULT_SET_CONTROL_MODE_SERVICE);
    const std::string gripper_command_topic = nhp.param(std::string("gripper_command_topic"), DEFAULT_GRIPPER_COMMAND_TOPIC);
    const std::string gripper_status_topic = nhp.param(std::string("gripper_status_topic"), DEFAULT_GRIPPER_STATUS_TOPIC);
    // Get LCM params
    const std::string send_lcm_url = nhp.param(std::string("send_lcm_url"), DEFAULT_SEND_LCM_URL);
    const std::string recv_lcm_url = nhp.param(std::string("recv_lcm_url"), DEFAULT_RECV_LCM_URL);
    const std::string motion_command_channel = nhp.param(std::string("motion_command_channel"), DEFAULT_MOTION_COMMAND_CHANNEL);
    const std::string motion_status_channel = nhp.param(std::string("motion_status_channel"), DEFAULT_MOTION_STATUS_CHANNEL);
    const std::string control_mode_command_channel = nhp.param(std::string("control_mode_command_channel"), DEFAULT_CONTROL_MODE_COMMAND_CHANNEL);
    const std::string control_mode_status_channel = nhp.param(std::string("control_mode_status_channel"), DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
    const std::string gripper_command_channel = nhp.param(std::string("gripper_command_channel"), DEFAULT_GRIPPER_COMMAND_CHANNEL);
    const std::string gripper_status_channel = nhp.param(std::string("gripper_status_channel"), DEFAULT_GRIPPER_STATUS_CHANNEL);
    const double set_control_mode_timeout = nhp.param(std::string("set_control_mode_timeout"), DEFAULT_SET_CONTROL_MODE_TIMEOUT);
    // Start LCM
    if (send_lcm_url == recv_lcm_url)
    {
        std::shared_ptr<lcm::LCM> lcm_ptr(new lcm::LCM(send_lcm_url));
        ROS_INFO_NAMED(ros::this_node::getName(), "Starting with shared send/receive LCM [%s]...", send_lcm_url.c_str());
        victor_hardware_interface::MinimalArmWrapperInterface interface(nh, cartesian_pose_frame, motion_command_topic, motion_status_topic, control_mode_status_topic, get_control_mode_service, set_control_mode_service, gripper_command_topic, gripper_status_topic, lcm_ptr, lcm_ptr, motion_command_channel, motion_status_channel, control_mode_command_channel, control_mode_status_channel, gripper_command_channel, gripper_status_channel, set_control_mode_timeout);
        interface.LCMLoop();
        return 0;
    }
    else
    {
        std::shared_ptr<lcm::LCM> send_lcm_ptr(new lcm::LCM(send_lcm_url));
        std::shared_ptr<lcm::LCM> recv_lcm_ptr(new lcm::LCM(recv_lcm_url));
        ROS_INFO_NAMED(ros::this_node::getName(), "Starting with separate send [%s] and receive [%s] LCM...", send_lcm_url.c_str(), recv_lcm_url.c_str());
        victor_hardware_interface::MinimalArmWrapperInterface interface(nh, cartesian_pose_frame, motion_command_topic, motion_status_topic, control_mode_status_topic, get_control_mode_service, set_control_mode_service, gripper_command_topic, gripper_status_topic, send_lcm_ptr, recv_lcm_ptr, motion_command_channel, motion_status_channel, control_mode_command_channel, control_mode_status_channel, gripper_command_channel, gripper_status_channel, set_control_mode_timeout);
        interface.LCMLoop();
        return 0;
    }
}
