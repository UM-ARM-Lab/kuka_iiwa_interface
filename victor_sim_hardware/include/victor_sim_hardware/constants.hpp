#pragma once

#include <chrono>
#include <string>

using namespace std::chrono_literals;

// Default ROS topic / service names (identical to real hardware)
const std::string DEFAULT_MOTION_STATUS_TOPIC("motion_status");
const std::string DEFAULT_CONTROL_MODE_PARAMETERS_TOPIC("control_mode_parameters");
const std::string DEFAULT_GRIPPER_COMMAND_TOPIC("gripper_command");
const std::string DEFAULT_GRIPPER_STATUS_TOPIC("gripper_status");

// Default simulator API channels (replacing LCM channels)
const std::string DEFAULT_MOTION_COMMAND_CHANNEL("motion_command");
const std::string DEFAULT_MOTION_STATUS_CHANNEL("motion_status");
const std::string DEFAULT_CONTROL_MODE_COMMAND_CHANNEL("control_mode_command");
const std::string DEFAULT_CONTROL_MODE_STATUS_CHANNEL("control_mode_status");
const std::string DEFAULT_GRIPPER_COMMAND_CHANNEL("gripper_command");
const std::string DEFAULT_GRIPPER_STATUS_CHANNEL("gripper_status");

// ROS2 command interfaces (identical to real hardware)
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

// ROS2 state interfaces (identical to real hardware)
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

// Force/Torque sensor state interfaces (identical to real hardware)
const std::string MEASURED_FX_STATE_INTERFACE = "measured/force_torque_sensor/fx";
const std::string MEASURED_FY_STATE_INTERFACE = "measured/force_torque_sensor/fy";
const std::string MEASURED_FZ_STATE_INTERFACE = "measured/force_torque_sensor/fz";
const std::string MEASURED_TX_STATE_INTERFACE = "measured/force_torque_sensor/tx";
const std::string MEASURED_TY_STATE_INTERFACE = "measured/force_torque_sensor/ty";
const std::string MEASURED_TZ_STATE_INTERFACE = "measured/force_torque_sensor/tz";

// Control mode command interfaces (identical to real hardware)
const std::string JOINT_POSITION_CONTROL_MODE_COMMAND_INTERFACE = "joint_position_control_mode";
const std::string JOINT_IMPEDANCE_CONTROL_MODE_COMMAND_INTERFACE = "joint_impedance_control_mode";
const std::string CARTESIAN_POSE_CONTROL_MODE_COMMAND_INTERFACE = "cartesian_pose_control_mode";
const std::string CARTESIAN_IMPEDANCE_CONTROL_MODE_COMMAND_INTERFACE = "cartesian_impedance_control_mode";

// Cartesian pose command interfaces (identical to real hardware)
const std::string COMMANDED_XT_COMMAND_INTERFACE = "commanded/cartesian_pose/xt";
const std::string COMMANDED_YT_COMMAND_INTERFACE = "commanded/cartesian_pose/yt";
const std::string COMMANDED_ZT_COMMAND_INTERFACE = "commanded/cartesian_pose/zt";
const std::string COMMANDED_WR_COMMAND_INTERFACE = "commanded/cartesian_pose/wr";
const std::string COMMANDED_XR_COMMAND_INTERFACE = "commanded/cartesian_pose/xr";
const std::string COMMANDED_YR_COMMAND_INTERFACE = "commanded/cartesian_pose/yr";
const std::string COMMANDED_ZR_COMMAND_INTERFACE = "commanded/cartesian_pose/zr";

// Control mode constants (matching LCM interface values)
namespace control_mode {
    constexpr int8_t JOINT_POSITION = 0;
    constexpr int8_t JOINT_IMPEDANCE = 1;
    constexpr int8_t CARTESIAN_POSE = 2;
    constexpr int8_t CARTESIAN_IMPEDANCE = 3;
}

// Default control mode parameters (identical values to real hardware)
constexpr auto DEFAULT_JOINT_RELATIVE_VELOCITY = 0.1;
constexpr auto DEFAULT_JOINT_RELATIVE_ACCELERATION = 0.1;
constexpr auto DEFAULT_OVERRIDE_JOINT_ACCELERATION = 0.0;
constexpr auto DEFAULT_JOINT_DAMPING = 1;
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
