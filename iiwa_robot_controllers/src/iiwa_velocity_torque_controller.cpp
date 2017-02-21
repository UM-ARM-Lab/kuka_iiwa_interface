#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <ros/ros.h>
#include <iiwa_robot_controllers/iiwa_robot_velocity_torque_controller.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_velocity_torque_controller");
    ROS_INFO("Starting iiwa_velocity_torque_controller...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC = "joint_command_velocity";
    const std::string DEFAULT_CONFIG_FEEDBACK_TOPIC = "joint_states";
    const std::string DEFAULT_TORQUE_COMMAND_TOPIC = "iiwa_FRI_command";
    const std::string DEFAULT_ABORT_SERVICE = "arm_velocity_torque_controller/abort";
    const std::string DEFAULT_JOINT_NAME_PREFIX = "iiwa";
    const double DEFAULT_CONTROL_RATE = 1000.0; //25.0;
    const bool DEFAULT_MODEL_GRAVITY = false;
    const int32_t DEFAULT_VELOCITY_FILTER_WINDOW_SIZE = 1;
    std::string torque_command_topic;
    std::string velocity_command_topic;
    std::string config_feedback_topic;
    std::string abort_service;
    std::string joint_name_prefix;
    double control_rate = DEFAULT_CONTROL_RATE;
    bool model_gravity = DEFAULT_MODEL_GRAVITY;
    int32_t velocity_filter_window_size = DEFAULT_VELOCITY_FILTER_WINDOW_SIZE;
    nhp.param(std::string("torque_command_topic"), torque_command_topic, DEFAULT_TORQUE_COMMAND_TOPIC);
    nhp.param(std::string("config_feedback_topic"), config_feedback_topic, DEFAULT_CONFIG_FEEDBACK_TOPIC);
    nhp.param(std::string("velocity_command_topic"), velocity_command_topic, DEFAULT_VELOCITY_COMMAND_TOPIC);
    nhp.param(std::string("abort_service"), abort_service, DEFAULT_ABORT_SERVICE);
    nhp.param(std::string("joint_name_prefix"), joint_name_prefix, DEFAULT_JOINT_NAME_PREFIX);
    nhp.param(std::string("control_rate"), control_rate, DEFAULT_CONTROL_RATE);
    nhp.param(std::string("model_gravity"), model_gravity, DEFAULT_MODEL_GRAVITY);
    nhp.param(std::string("velocity_filter_window_size"), velocity_filter_window_size, DEFAULT_VELOCITY_FILTER_WINDOW_SIZE);
    // Get the XML string of the URDF
    std::string xml_model_string;
    nh.param(std::string("robot_description"), xml_model_string, std::string(""));
    // Joint limits
    const std::map<std::string, iiwa_robot_controllers::JointLimits> joint_limits = iiwa_robot_controllers::GetArmLimits(joint_name_prefix, 0.95, 0.10, 0.50);
    // Joint PID params
    const std::map<std::string, iiwa_robot_controllers::PIDParams> joint_controller_params = iiwa_robot_controllers::GetArmDefaultVelocityControllerParams(joint_name_prefix);
    // Assemble the controller
    iiwa_robot_controllers::IIWARobotVelocityTorqueController controller(nh, velocity_command_topic, config_feedback_topic, torque_command_topic, abort_service, xml_model_string, model_gravity, velocity_filter_window_size, joint_limits, joint_controller_params);
    ROS_INFO("...startup complete");
    controller.Loop(control_rate);
    return 0;
}
