#pragma once

#include <mutex>
#include <string>
#include <thread>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <victor_hardware_interfaces/msg/cartesian_control_mode_limits.hpp>
#include <victor_hardware_interfaces/msg/cartesian_impedance_parameters.hpp>
#include <victor_hardware_interfaces/msg/cartesian_path_execution_parameters.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/motion_command.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>
#include <victor_hardware_interfaces/srv/get_control_mode.hpp>
#include <victor_hardware_interfaces/srv/set_control_mode.hpp>


namespace victor_hardware {
namespace msg = victor_hardware_interfaces::msg;
namespace srv = victor_hardware_interfaces::srv;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers to test if two messages are equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool jvqEqual(const msg::JointValueQuantity& jvq1, const msg::JointValueQuantity& jvq2);

bool cvqEqual(const msg::CartesianValueQuantity& cvq1, const msg::CartesianValueQuantity& cvq2);

bool jointPexpEqual(const msg::JointPathExecutionParameters& pexp1, const msg::JointPathExecutionParameters& pexp2);

bool cartesianPexpEqual(const msg::CartesianPathExecutionParameters& pexp1,
                        const msg::CartesianPathExecutionParameters& pexp2);

bool controlModeParamsEqual(const msg::ControlModeParameters& params1, const msg::ControlModeParameters& params2);

}  // namespace victor_hardware