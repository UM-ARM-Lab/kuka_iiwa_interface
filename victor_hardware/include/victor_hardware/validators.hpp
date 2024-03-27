#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
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

using ErrorType = std::pair<bool, std::string>;

[[nodiscard]] ErrorType validateJointPathExecutionParams(const victor_lcm_interface::joint_path_execution_parameters& params);

[[nodiscard]] ErrorType validateCartesianPathExecutionParams(const victor_lcm_interface::cartesian_path_execution_parameters& params);

[[nodiscard]] ErrorType validateJointImpedanceParams(const victor_lcm_interface::joint_impedance_parameters& params);

[[nodiscard]] ErrorType validateCartesianImpedanceParams(const victor_lcm_interface::cartesian_impedance_parameters& params);

[[nodiscard]] ErrorType validateCartesianControlModeLimits(const victor_lcm_interface::cartesian_control_mode_limits& params);

[[nodiscard]] ErrorType validateControlMode(const victor_lcm_interface::control_mode_parameters& params);

[[nodiscard]] ErrorType validateMotionCommand(const victor_lcm_interface::motion_command& command);

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