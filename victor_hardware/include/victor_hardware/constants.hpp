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

// ROS2 command interfaces
static const char *const COMMANDED_POSITION = "commanded_position";
static const char *const EXTERNAL_TORQUE = "external_torque";
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

// ROS2 state interfaces
const std::string MEASURED_XT_STATE_INTERFACE = "measured/cartesian_pose/xt";
const std::string MEASURED_YT_STATE_INTERFACE = "measured/cartesian_pose/yt";
const std::string MEASURED_ZT_STATE_INTERFACE = "measured/cartesian_pose/zt";
const std::string MEASURED_WR_STATE_INTERFACE = "measured/cartesian_pose/wr";
const std::string MEASURED_XR_STATE_INTERFACE = "measured/cartesian_pose/xr";
const std::string MEASURED_YR_STATE_INTERFACE = "measured/cartesian_pose/yr";
const std::string MEASURED_ZR_STATE_INTERFACE = "measured/cartesian_pose/zr";
const std::string COMMANDED_XT_STATE_INTERFACE = "commanded/cartesian_pose/xt";
const std::string COMMANDED_YT_STATE_INTERFACE = "commanded/cartesian_pose/yt";
const std::string COMMANDED_ZT_STATE_INTERFACE = "commanded/cartesian_pose/zt";
const std::string COMMANDED_WR_STATE_INTERFACE = "commanded/cartesian_pose/wr";
const std::string COMMANDED_XR_STATE_INTERFACE = "commanded/cartesian_pose/xr";
const std::string COMMANDED_YR_STATE_INTERFACE = "commanded/cartesian_pose/yr";
const std::string COMMANDED_ZR_STATE_INTERFACE = "commanded/cartesian_pose/zr";

// Define constants for the control mode parameters, so we can initialize them the same way
// both in default_control_mode_parameters() and the ROS params in the control_mode_params_helper
constexpr auto DEFAULT_JOINT_RELATIVE_VELOCITY = 0.1;
constexpr auto DEFAULT_JOINT_RELATIVE_ACCELERATION = 0.1;
constexpr auto DEFAULT_OVERRIDE_JOINT_ACCELERATION = 0.0;
constexpr auto DEFAULT_JOINT_DAMPING = 0.7;
constexpr auto DEFAULT_JOINT1_STIFFNESS = 600.0;
constexpr auto DEFAULT_JOINT2_STIFFNESS = 600.0;
constexpr auto DEFAULT_JOINT3_STIFFNESS = 300.0;
constexpr auto DEFAULT_JOINT4_STIFFNESS = 300.0;
constexpr auto DEFAULT_JOINT5_STIFFNESS = 100.0;
constexpr auto DEFAULT_JOINT6_STIFFNESS = 100.0;
constexpr auto DEFAULT_JOINT7_STIFFNESS = 50.0;
constexpr auto DEFAULT_MAX_LIN_VELOCITY = 75.0;
constexpr auto DEFAULT_MAX_ROT_VELOCITY = 25.0;
constexpr auto DEFAULT_MAX_NULLSPACE_VELOCITY = 750.0;
constexpr auto DEFAULT_MAX_LIN_ACCELERATION = 0.1;
constexpr auto DEFAULT_MAX_ROT_ACCELERATION = 0.1;
constexpr auto DEFAULT_MAX_NULLSPACE_ACCELERATION = 1.0;
constexpr auto DEFAULT_CARTESIAN_DAMPING = 0.25;
constexpr auto DEFAULT_NULLSPACE_DAMPING = 0.75;
constexpr auto DEFAULT_CARTESIAN_STIFFNESS = 5000.0;
constexpr auto DEFAULT_ROT_STIFFNESS = 300.0;
constexpr auto DEFAULT_NULLSPACE_STIFFNESS = 100.0;
constexpr auto DEFAULT_MAX_PATH_DEVIATION = 10000000.0;
constexpr auto DEFAULT_MAX_CARTESIAN_LIN_VELOCITY = 75.0;
constexpr auto DEFAULT_MAX_CARTESIAN_ROT_VELOCITY = 40.0;
constexpr auto DEFAULT_MAX_CONTROL_FORCE = 20.0;
constexpr auto DEFAULT_STOP_ON_MAX_CONTROL_FORCE = false;

constexpr victor_lcm_interface::control_mode_parameters default_control_mode_parameters() {
    victor_lcm_interface::control_mode_parameters params{};

    // These should probably match the values in victor_utils.py
    params.joint_path_execution_params.joint_relative_velocity = DEFAULT_JOINT_RELATIVE_VELOCITY;
    params.joint_path_execution_params.joint_relative_acceleration = DEFAULT_JOINT_RELATIVE_ACCELERATION;
    params.joint_path_execution_params.override_joint_acceleration = DEFAULT_OVERRIDE_JOINT_ACCELERATION;

    params.joint_impedance_params.joint_damping.joint_1 = DEFAULT_JOINT_DAMPING;
    params.joint_impedance_params.joint_damping.joint_2 = DEFAULT_JOINT_DAMPING;
    params.joint_impedance_params.joint_damping.joint_3 = DEFAULT_JOINT_DAMPING;
    params.joint_impedance_params.joint_damping.joint_4 = DEFAULT_JOINT_DAMPING;
    params.joint_impedance_params.joint_damping.joint_5 = DEFAULT_JOINT_DAMPING;
    params.joint_impedance_params.joint_damping.joint_6 = DEFAULT_JOINT_DAMPING;
    params.joint_impedance_params.joint_damping.joint_7 = DEFAULT_JOINT_DAMPING;

    params.joint_impedance_params.joint_stiffness.joint_1 = DEFAULT_JOINT1_STIFFNESS;
    params.joint_impedance_params.joint_stiffness.joint_2 = DEFAULT_JOINT2_STIFFNESS;
    params.joint_impedance_params.joint_stiffness.joint_3 = DEFAULT_JOINT3_STIFFNESS;
    params.joint_impedance_params.joint_stiffness.joint_4 = DEFAULT_JOINT4_STIFFNESS;
    params.joint_impedance_params.joint_stiffness.joint_5 = DEFAULT_JOINT5_STIFFNESS;
    params.joint_impedance_params.joint_stiffness.joint_6 = DEFAULT_JOINT6_STIFFNESS;
    params.joint_impedance_params.joint_stiffness.joint_7 = DEFAULT_JOINT7_STIFFNESS;

    params.cartesian_path_execution_params.max_velocity.x = DEFAULT_MAX_LIN_VELOCITY;
    params.cartesian_path_execution_params.max_velocity.y = DEFAULT_MAX_LIN_VELOCITY;
    params.cartesian_path_execution_params.max_velocity.z = DEFAULT_MAX_LIN_VELOCITY;
    params.cartesian_path_execution_params.max_velocity.a = DEFAULT_MAX_ROT_VELOCITY;
    params.cartesian_path_execution_params.max_velocity.b = DEFAULT_MAX_ROT_VELOCITY;
    params.cartesian_path_execution_params.max_velocity.c = DEFAULT_MAX_ROT_VELOCITY;
    params.cartesian_path_execution_params.max_nullspace_velocity = DEFAULT_MAX_NULLSPACE_VELOCITY;
    params.cartesian_path_execution_params.max_acceleration.x = DEFAULT_MAX_LIN_ACCELERATION;
    params.cartesian_path_execution_params.max_acceleration.y = DEFAULT_MAX_LIN_ACCELERATION;
    params.cartesian_path_execution_params.max_acceleration.z = DEFAULT_MAX_LIN_ACCELERATION;
    params.cartesian_path_execution_params.max_acceleration.a = DEFAULT_MAX_ROT_ACCELERATION;
    params.cartesian_path_execution_params.max_acceleration.b = DEFAULT_MAX_ROT_ACCELERATION;
    params.cartesian_path_execution_params.max_acceleration.c = DEFAULT_MAX_ROT_ACCELERATION;
    params.cartesian_path_execution_params.max_nullspace_acceleration = DEFAULT_MAX_NULLSPACE_ACCELERATION;

    params.cartesian_impedance_params.cartesian_damping.x = DEFAULT_CARTESIAN_DAMPING;
    params.cartesian_impedance_params.cartesian_damping.y = DEFAULT_CARTESIAN_DAMPING;
    params.cartesian_impedance_params.cartesian_damping.z = DEFAULT_CARTESIAN_DAMPING;
    params.cartesian_impedance_params.cartesian_damping.a = DEFAULT_CARTESIAN_DAMPING;
    params.cartesian_impedance_params.cartesian_damping.b = DEFAULT_CARTESIAN_DAMPING;
    params.cartesian_impedance_params.cartesian_damping.c = DEFAULT_CARTESIAN_DAMPING;
    params.cartesian_impedance_params.nullspace_damping = DEFAULT_NULLSPACE_DAMPING;
    params.cartesian_impedance_params.cartesian_stiffness.x = DEFAULT_CARTESIAN_STIFFNESS;
    params.cartesian_impedance_params.cartesian_stiffness.y = DEFAULT_CARTESIAN_STIFFNESS;
    params.cartesian_impedance_params.cartesian_stiffness.z = DEFAULT_CARTESIAN_STIFFNESS;
    params.cartesian_impedance_params.cartesian_stiffness.a = DEFAULT_ROT_STIFFNESS;
    params.cartesian_impedance_params.cartesian_stiffness.b = DEFAULT_ROT_STIFFNESS;
    params.cartesian_impedance_params.cartesian_stiffness.c = DEFAULT_ROT_STIFFNESS;
    params.cartesian_impedance_params.nullspace_stiffness = DEFAULT_NULLSPACE_STIFFNESS;

    params.cartesian_control_mode_limits.max_path_deviation.x = DEFAULT_MAX_PATH_DEVIATION;
    params.cartesian_control_mode_limits.max_path_deviation.y = DEFAULT_MAX_PATH_DEVIATION;
    params.cartesian_control_mode_limits.max_path_deviation.z = DEFAULT_MAX_PATH_DEVIATION;
    params.cartesian_control_mode_limits.max_path_deviation.a = DEFAULT_MAX_PATH_DEVIATION;
    params.cartesian_control_mode_limits.max_path_deviation.b = DEFAULT_MAX_PATH_DEVIATION;
    params.cartesian_control_mode_limits.max_path_deviation.c = DEFAULT_MAX_PATH_DEVIATION;
    params.cartesian_control_mode_limits.max_cartesian_velocity.x = DEFAULT_MAX_CARTESIAN_LIN_VELOCITY;
    params.cartesian_control_mode_limits.max_cartesian_velocity.y = DEFAULT_MAX_CARTESIAN_LIN_VELOCITY;
    params.cartesian_control_mode_limits.max_cartesian_velocity.z = DEFAULT_MAX_CARTESIAN_LIN_VELOCITY;
    params.cartesian_control_mode_limits.max_cartesian_velocity.a = DEFAULT_MAX_CARTESIAN_ROT_VELOCITY;
    params.cartesian_control_mode_limits.max_cartesian_velocity.b = DEFAULT_MAX_CARTESIAN_ROT_VELOCITY;
    params.cartesian_control_mode_limits.max_cartesian_velocity.c = DEFAULT_MAX_CARTESIAN_ROT_VELOCITY;
    params.cartesian_control_mode_limits.max_control_force.x = DEFAULT_MAX_CONTROL_FORCE;
    params.cartesian_control_mode_limits.max_control_force.y = DEFAULT_MAX_CONTROL_FORCE;
    params.cartesian_control_mode_limits.max_control_force.z = DEFAULT_MAX_CONTROL_FORCE;
    params.cartesian_control_mode_limits.max_control_force.a = DEFAULT_MAX_CONTROL_FORCE;
    params.cartesian_control_mode_limits.max_control_force.b = DEFAULT_MAX_CONTROL_FORCE;
    params.cartesian_control_mode_limits.max_control_force.c = DEFAULT_MAX_CONTROL_FORCE;
    params.cartesian_control_mode_limits.stop_on_max_control_force = DEFAULT_STOP_ON_MAX_CONTROL_FORCE;

    return params;
}