#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/motion_command.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>
#include <victor_hardware_interfaces/srv/get_control_mode.hpp>
#include <victor_hardware_interfaces/srv/set_control_mode.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>

namespace msg = victor_hardware_interfaces::msg;
namespace srv = victor_hardware_interfaces::srv;

namespace victor_hardware {

std::pair<bool, std::string> validateJointPathExecutionParams(const msg::JointPathExecutionParameters& params);

std::pair<bool, std::string> validateCartesianPathExecutionParams(const msg::CartesianPathExecutionParameters& params);

std::pair<bool, std::string> validateJointImpedanceParams(const msg::JointImpedanceParameters& params);

std::pair<bool, std::string> validateCartesianImpedanceParams(const msg::CartesianImpedanceParameters& params);

std::pair<bool, std::string> validateCartesianControlModeLimits(const msg::CartesianControlModeLimits& params);

std::pair<bool, std::string> validateControlMode(const msg::ControlModeParameters& params);

std::pair<bool, std::string> validateCartesianPose(const geometry_msgs::msg::Pose& pose, const std::string& frame);

std::pair<bool, std::string> validateMotionCommand(uint8_t active_control_mode,
                                                   const victor_lcm_interface::motion_command& command);

std::pair<bool, std::string> validateGripperCommand(const msg::Robotiq3FingerCommand& command);

bool jointPathExecutionParamsIsDefault(const msg::JointPathExecutionParameters& params);

bool cartesianPathExecutionParamsIsDefault(const msg::CartesianPathExecutionParameters& params);

bool jointImpedanceParamsIsDefault(const msg::JointImpedanceParameters& params);

bool cartesianImpedanceParamsIsDefault(const msg::CartesianImpedanceParameters& params);

bool cartesianControlModeLimitsIsDefault(const msg::CartesianControlModeLimits& params);

msg::ControlModeParameters mergeControlModeParameters(const msg::ControlModeParameters& active_control_mode,
                                                      const msg::ControlModeParameters& new_control_mode);

bool operator==(victor_lcm_interface::control_mode_parameters const& p1,
                victor_lcm_interface::control_mode_parameters const& p2);

bool operator==(victor_lcm_interface::control_mode const& p1, victor_lcm_interface::control_mode const& p2);

bool operator==(victor_lcm_interface::joint_impedance_parameters const& p1,
                victor_lcm_interface::joint_impedance_parameters const& p2);

bool operator==(victor_lcm_interface::cartesian_impedance_parameters const& p1,
                victor_lcm_interface::cartesian_impedance_parameters const& p2);

bool operator==(victor_lcm_interface::cartesian_control_mode_limits const& p1,
                victor_lcm_interface::cartesian_control_mode_limits const& p2);

bool operator==(victor_lcm_interface::cartesian_path_execution_parameters const& p1,
                victor_lcm_interface::cartesian_path_execution_parameters const& p2);

bool operator==(victor_lcm_interface::joint_path_execution_parameters const& p1,
                victor_lcm_interface::joint_path_execution_parameters const& p2);

bool operator==(victor_lcm_interface::joint_value_quantity const& p1,
                victor_lcm_interface::joint_value_quantity const& p2);

bool operator==(victor_lcm_interface::cartesian_pose const& p1, victor_lcm_interface::cartesian_pose const& p2);

bool operator==(victor_lcm_interface::cartesian_value_quantity const& p1,
                victor_lcm_interface::cartesian_value_quantity const& p2);

}  // namespace victor_hardware