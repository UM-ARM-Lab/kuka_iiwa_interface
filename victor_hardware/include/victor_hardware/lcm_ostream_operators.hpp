#pragma once

#include <iostream>
#include <victor_lcm_interface/control_mode.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_lcm_interface/motion_command.hpp>
#include <victor_lcm_interface/motion_status.hpp>
#include <victor_lcm_interface/robotiq_3finger_command.hpp>
#include <victor_lcm_interface/robotiq_3finger_status.hpp>

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_control_mode_limits& limits);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_impedance_parameters& params);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_path_execution_parameters& params);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_pose& pose);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_value_quantity& cvq);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::control_mode& mode);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::control_mode_parameters& params);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::joint_impedance_parameters& params);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::joint_path_execution_parameters& params);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::joint_value_quantity& jvq);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::motion_command& cmd);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::motion_status& status);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::robotiq_3finger_command& cmd);
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::robotiq_3finger_status& status);