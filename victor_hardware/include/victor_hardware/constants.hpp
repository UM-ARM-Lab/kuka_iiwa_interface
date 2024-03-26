#pragma once

#include <chrono>
#include <string>
#include <victor_lcm_interface/control_mode_parameters.hpp>

using namespace std::chrono_literals;

// NOTE: changing these values here requires corresponding changes in the LCMRobotInterface application
//  and also the victor_lcm_bridge launch file.
std::string const LEFT_RECV_PROVIDER = "udp://10.10.10.169:30002";
std::string const RIGHT_RECV_PROVIDER = "udp://10.10.10.169:30001";
std::string const LEFT_SEND_PROVIDER = "udp://10.10.10.12:30000";
std::string const RIGHT_SEND_PROVIDER = "udp://10.10.10.11:30000";

const std::string DEFAULT_CARTESIAN_CONTROL_FRAME = "base";
const auto DEFAULT_SET_CONTROL_MODE_TIMEOUT = 2.5s;

// Default ROS topic / service names
const std::string DEFAULT_MOTION_COMMAND_TOPIC("motion_command");
const std::string DEFAULT_MOTION_STATUS_TOPIC("motion_status");
const std::string DEFAULT_CONTROL_MODE_PARAMETERS_TOPIC("control_mode_parameters");
const std::string DEFAULT_GRIPPER_COMMAND_TOPIC("gripper_command");
const std::string DEFAULT_GRIPPER_STATUS_TOPIC("gripper_status");

// Default LCM parameters
const std::string DEFAULT_MOTION_COMMAND_CHANNEL("motion_command");
const std::string DEFAULT_MOTION_STATUS_CHANNEL("motion_status");
const std::string DEFAULT_CONTROL_MODE_COMMAND_CHANNEL("control_mode_command");
const std::string DEFAULT_CONTROL_MODE_STATUS_CHANNEL("control_mode_status");
const std::string DEFAULT_GRIPPER_COMMAND_CHANNEL("gripper_command");
const std::string DEFAULT_GRIPPER_STATUS_CHANNEL("gripper_status");

// ROS2 control interface
static const char* const COMMANDED_POSITION = "commanded_position";
static const char* const EXTERNAL_TORQUE = "external_torque";
const std::string CARTESIAN_XT_INTERFACE = "cartesian_pose/xt";
const std::string CARTESIAN_YT_INTERFACE = "cartesian_pose/yt";
const std::string CARTESIAN_ZT_INTERFACE = "cartesian_pose/zt";
const std::string CARTESIAN_WR_INTERFACE = "cartesian_pose/wr";
const std::string CARTESIAN_XR_INTERFACE = "cartesian_pose/xr";
const std::string CARTESIAN_YR_INTERFACE = "cartesian_pose/yr";
const std::string CARTESIAN_ZR_INTERFACE = "cartesian_pose/zr";
const std::string JOINT_POSITION_INTERFACE = "joint_position";
const std::string JOINT_IMPEDANCE_INTERFACE = "joint_impedance";
const std::string CARTESIAN_POSE_INTERFACE = "cartesian_pose";
const std::string CARTESIAN_IMPEDANCE_INTERFACE = "cartesian_impedance";

constexpr victor_lcm_interface::control_mode_parameters default_control_mode_parameters() {
  victor_lcm_interface::control_mode_parameters params{};

  // These should probably match the values in victor_utils.py
  params.joint_path_execution_params.joint_relative_velocity = 0.1;
  params.joint_path_execution_params.joint_relative_acceleration = 0.1;
  params.joint_path_execution_params.override_joint_acceleration = 0.0;

  params.joint_impedance_params.joint_damping.joint_1 = 0.7;
  params.joint_impedance_params.joint_damping.joint_2 = 0.7;
  params.joint_impedance_params.joint_damping.joint_3 = 0.7;
  params.joint_impedance_params.joint_damping.joint_4 = 0.7;
  params.joint_impedance_params.joint_damping.joint_5 = 0.7;
  params.joint_impedance_params.joint_damping.joint_6 = 0.7;
  params.joint_impedance_params.joint_damping.joint_7 = 0.7;

  params.joint_impedance_params.joint_stiffness.joint_1 = 600.0;
  params.joint_impedance_params.joint_stiffness.joint_2 = 600.0;
  params.joint_impedance_params.joint_stiffness.joint_3 = 300.0;
  params.joint_impedance_params.joint_stiffness.joint_4 = 300.0;
  params.joint_impedance_params.joint_stiffness.joint_5 = 100.0;
  params.joint_impedance_params.joint_stiffness.joint_6 = 100.0;
  params.joint_impedance_params.joint_stiffness.joint_7 = 50.0;

  params.cartesian_path_execution_params.max_velocity.x = 75.0;
  params.cartesian_path_execution_params.max_velocity.y = 75.0;
  params.cartesian_path_execution_params.max_velocity.z = 75.0;
  params.cartesian_path_execution_params.max_velocity.a = 75.0 * 0.25;
  params.cartesian_path_execution_params.max_velocity.b = 75.0 * 0.25;
  params.cartesian_path_execution_params.max_velocity.c = 75.0 * 0.25;
  params.cartesian_path_execution_params.max_nullspace_velocity = 750.0;
  params.cartesian_path_execution_params.max_acceleration.x = 0.1;
  params.cartesian_path_execution_params.max_acceleration.y = 0.1;
  params.cartesian_path_execution_params.max_acceleration.z = 0.1;
  params.cartesian_path_execution_params.max_acceleration.a = 0.1;
  params.cartesian_path_execution_params.max_acceleration.b = 0.1;
  params.cartesian_path_execution_params.max_acceleration.c = 0.1;

  params.cartesian_impedance_params.cartesian_damping.x = 0.25;
  params.cartesian_impedance_params.cartesian_damping.y = 0.25;
  params.cartesian_impedance_params.cartesian_damping.z = 0.25;
  params.cartesian_impedance_params.cartesian_damping.a = 0.25;
  params.cartesian_impedance_params.cartesian_damping.b = 0.25;
  params.cartesian_impedance_params.cartesian_damping.c = 0.25;
  params.cartesian_impedance_params.nullspace_damping = 0.75;
  params.cartesian_impedance_params.cartesian_stiffness.x = 5000.0;
  params.cartesian_impedance_params.cartesian_stiffness.y = 5000.0;
  params.cartesian_impedance_params.cartesian_stiffness.z = 5000.0;
  params.cartesian_impedance_params.cartesian_stiffness.a = 300.0;
  params.cartesian_impedance_params.cartesian_stiffness.b = 300.0;
  params.cartesian_impedance_params.cartesian_stiffness.c = 300.0;
  params.cartesian_impedance_params.nullspace_stiffness = 100.0;

  params.cartesian_control_mode_limits.max_path_deviation.x = 10000000.0;
  params.cartesian_control_mode_limits.max_path_deviation.y = 10000000.0;
  params.cartesian_control_mode_limits.max_path_deviation.z = 10000000.0;
  params.cartesian_control_mode_limits.max_path_deviation.a = 10000000.0;
  params.cartesian_control_mode_limits.max_path_deviation.b = 10000000.0;
  params.cartesian_control_mode_limits.max_path_deviation.c = 10000000.0;
  params.cartesian_control_mode_limits.max_cartesian_velocity.x = 75.0;
  params.cartesian_control_mode_limits.max_cartesian_velocity.y = 75.0;
  params.cartesian_control_mode_limits.max_cartesian_velocity.z = 75.0;
  params.cartesian_control_mode_limits.max_cartesian_velocity.a = 75.0 * 2.0 * 0.25;
  params.cartesian_control_mode_limits.max_cartesian_velocity.b = 75.0 * 2.0 * 0.25;
  params.cartesian_control_mode_limits.max_cartesian_velocity.c = 75.0 * 2.0 * 0.25;
  params.cartesian_control_mode_limits.max_control_force.x = 20.0;
  params.cartesian_control_mode_limits.max_control_force.y = 20.0;
  params.cartesian_control_mode_limits.max_control_force.z = 20.0;
  params.cartesian_control_mode_limits.max_control_force.a = 20.0;
  params.cartesian_control_mode_limits.max_control_force.b = 20.0;
  params.cartesian_control_mode_limits.max_control_force.c = 20.0;
  params.cartesian_control_mode_limits.stop_on_max_control_force = false;

  return params;
}