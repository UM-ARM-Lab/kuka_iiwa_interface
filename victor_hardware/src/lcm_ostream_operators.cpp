#include <victor_hardware/lcm_ostream_operators.hpp>

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::control_mode_parameters& params) {
  os << "control_mode_parameters: {";
  os << "joint_impedance_params: " << params.joint_impedance_params << "\n";
  os << "cartesian_impedance_params: " << params.cartesian_impedance_params << "\n";
  os << "cartesian_control_mode_limits: " << params.cartesian_control_mode_limits << "\n";
  os << "joint_path_execution_params: " << params.joint_path_execution_params << "\n";
  os << "cartesian_path_execution_params: " << params.cartesian_path_execution_params << "\n";
  os << "control_mode: " << params.control_mode << "\n";
  os << "}";
  return os;
}
std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::control_mode& mode) {
  os << "control_mode: {";
  os << "mode: " << mode.mode << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::joint_impedance_parameters& params) {
  os << "joint_impedance_params: {";
  os << "stiffness: " << params.joint_stiffness << "\n";
  os << "damping: " << params.joint_damping << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::joint_value_quantity& jvq) {
  os << "joint_value_quantity: {";
  os << "joint_1: " << jvq.joint_1 << "\n";
  os << "joint_2: " << jvq.joint_2 << "\n";
  os << "joint_3: " << jvq.joint_3 << "\n";
  os << "joint_4: " << jvq.joint_4 << "\n";
  os << "joint_5: " << jvq.joint_5 << "\n";
  os << "joint_6: " << jvq.joint_6 << "\n";
  os << "joint_7: " << jvq.joint_7 << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_pose& pose) {
  os << "cartesian_pose: {";
  os << "xt: " << pose.xt << "\n";
  os << "yt: " << pose.yt << "\n";
  os << "zt: " << pose.zt << "\n";
  os << "wr: " << pose.wr << "\n";
  os << "xr: " << pose.xr << "\n";
  os << "yr: " << pose.yr << "\n";
  os << "zr: " << pose.zr << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_impedance_parameters& params) {
  os << "cartesian_impedance_params: {";
  os << "stiffness: " << params.cartesian_stiffness << "\n";
  os << "damping: " << params.cartesian_damping << "\n";
  os << "nullspace_stiffness: " << params.nullspace_stiffness << "\n";
  os << "nullspace_damping: " << params.nullspace_damping << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_value_quantity& cvq) {
  os << "cartesian_value_quantity: {";
  os << "x: " << cvq.x << "\n";
  os << "y: " << cvq.y << "\n";
  os << "z: " << cvq.z << "\n";
  os << "a: " << cvq.a << "\n";
  os << "b: " << cvq.b << "\n";
  os << "c: " << cvq.c << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_control_mode_limits& limits) {
  os << "cartesian_control_mode_limits: {";
  os << "max_velocity: " << limits.max_cartesian_velocity << "\n";
  os << "max_path_deviation: " << limits.max_path_deviation << "\n";
  os << "max_control_force: " << limits.max_control_force << "\n";
  os << "stop_on_max_control_force: " << limits.stop_on_max_control_force << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::joint_path_execution_parameters& params) {
  os << "joint_path_execution_params: {";
  os << "joint_relative_velocity: " << params.joint_relative_velocity << "\n";
  os << "joint_relative_acceleration: " << params.joint_relative_acceleration << "\n";
  os << "override_joint_acceleration: " << params.override_joint_acceleration << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::cartesian_path_execution_parameters& params) {
  os << "cartesian_path_execution_params: {";
  os << "max_velocity: " << params.max_velocity << "\n";
  os << "max_acceleration: " << params.max_acceleration << "\n";
  os << "max_nullspace_velocity: " << params.max_nullspace_velocity << "\n";
  os << "max_nullspace_acceleration: " << params.max_nullspace_acceleration << "\n";
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const victor_lcm_interface::motion_command& cmd) {
  os << "motion_command: {";
  os << "joint_position: " << cmd.joint_position << "\n";
  os << "joint_velocity: " << cmd.joint_velocity << "\n";
  os << "cartesian_pose: " << cmd.cartesian_pose << "\n";
  os << "control_mode: " << cmd.control_mode << "\n";
  os << "}";
  return os;
}