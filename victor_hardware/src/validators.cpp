#include <cmath>
#include <optional>
#include <rclcpp/logger.hpp>
#include <victor_hardware/validators.hpp>
#include <victor_hardware/constants.hpp>

static auto logger = rclcpp::get_logger("validators");

namespace victor_hardware {

std::pair<bool, std::string> validateJointPathExecutionParams(const msg::JointPathExecutionParameters& params) {
  bool valid = true;
  std::string message;
  if (params.joint_relative_velocity <= 0.0 || params.joint_relative_velocity > 1.0) {
    valid = false;
    message += "+Invalid joint relative velocity";
  }
  if (params.joint_relative_acceleration <= 0.0 || params.joint_relative_acceleration > 1.0) {
    valid = false;
    message += "+Invalid joint relative acceleration";
  }
  if (params.override_joint_acceleration < 0.0 || params.override_joint_acceleration > 10.0) {
    valid = false;
    message += "+Invalid override joint acceleration";
  }
  return {valid, message};
}

std::pair<bool, std::string> validateCartesianPathExecutionParams(const msg::CartesianPathExecutionParameters& params) {
  bool valid = true;
  std::string message;

  // Velocities, mm/s, rad/s, and 1/s respectively
  if (params.max_velocity.x <= 0.0) {
    valid = false;
    message += "+Invalid DoF X max velocity";
  }
  if (params.max_velocity.y <= 0.0) {
    valid = false;
    message += "+Invalid DoF Y max velocity";
  }
  if (params.max_velocity.z <= 0.0) {
    valid = false;
    message += "+Invalid DoF Z max velocity";
  }
  if (params.max_velocity.a <= 0.0) {
    valid = false;
    message += "+Invalid DoF A max velocity";
  }
  if (params.max_velocity.b <= 0.0) {
    valid = false;
    message += "+Invalid DoF B max velocity";
  }
  if (params.max_velocity.c <= 0.0) {
    valid = false;
    message += "+Invalid DoF C max velocity";
  }
  if (params.max_nullspace_velocity <= 0.0) {
    valid = false;
    message += "+Invalid nullspace max velocity";
  }

  // Accelerations, mm/s^2, rad/s^2, and 1/s^2 respectively
  if (params.max_acceleration.x <= 0.0) {
    valid = false;
    message += "+Invalid DoF X max acceleration";
  }
  if (params.max_acceleration.y <= 0.0) {
    valid = false;
    message += "+Invalid DoF Y max acceleration";
  }
  if (params.max_acceleration.z <= 0.0) {
    valid = false;
    message += "+Invalid DoF Z max acceleration";
  }
  if (params.max_acceleration.a <= 0.0) {
    valid = false;
    message += "+Invalid DoF A max acceleration";
  }
  if (params.max_acceleration.b <= 0.0) {
    valid = false;
    message += "+Invalid DoF B max acceleration";
  }
  if (params.max_acceleration.c <= 0.0) {
    valid = false;
    message += "+Invalid DoF C max acceleration";
  }
  if (params.max_nullspace_acceleration <= 0.0) {
    valid = false;
    message += "+Invalid nullspace max acceleration";
  }

  return {valid, message};
}

std::pair<bool, std::string> validateJointImpedanceParams(const msg::JointImpedanceParameters& params) {
  bool valid = true;
  std::string message;

  // Joint damping - unitless
  if (params.joint_damping.joint_1 < 0.0 || params.joint_damping.joint_1 > 1.0) {
    valid = false;
    message += "+Invalid joint 1 damping";
  }
  if (params.joint_damping.joint_2 < 0.0 || params.joint_damping.joint_2 > 1.0) {
    valid = false;
    message += "+Invalid joint 2 damping";
  }
  if (params.joint_damping.joint_3 < 0.0 || params.joint_damping.joint_3 > 1.0) {
    valid = false;
    message += "+Invalid joint 3 damping";
  }
  if (params.joint_damping.joint_4 < 0.0 || params.joint_damping.joint_4 > 1.0) {
    valid = false;
    message += "+Invalid joint 4 damping";
  }
  if (params.joint_damping.joint_5 < 0.0 || params.joint_damping.joint_5 > 1.0) {
    valid = false;
    message += "+Invalid joint 5 damping";
  }
  if (params.joint_damping.joint_6 < 0.0 || params.joint_damping.joint_6 > 1.0) {
    valid = false;
    message += "+Invalid joint 6 damping";
  }
  if (params.joint_damping.joint_7 < 0.0 || params.joint_damping.joint_7 > 1.0) {
    valid = false;
    message += "+Invalid joint 7 damping";
  }

  // Joint stiffness - Nm/rad
  if (params.joint_stiffness.joint_1 < 0.0) {
    valid = false;
    message += "+Invalid joint 1 stiffness";
  }
  if (params.joint_stiffness.joint_2 < 0.0) {
    valid = false;
    message += "+Invalid joint 2 stiffness";
  }
  if (params.joint_stiffness.joint_3 < 0.0) {
    valid = false;
    message += "+Invalid joint 3 stiffness";
  }
  if (params.joint_stiffness.joint_4 < 0.0) {
    valid = false;
    message += "+Invalid joint 4 stiffness";
  }
  if (params.joint_stiffness.joint_5 < 0.0) {
    valid = false;
    message += "+Invalid joint 5 stiffness";
  }
  if (params.joint_stiffness.joint_6 < 0.0) {
    valid = false;
    message += "+Invalid joint 6 stiffness";
  }
  if (params.joint_stiffness.joint_7 < 0.0) {
    valid = false;
    message += "+Invalid joint 7 stiffness";
  }

  return {valid, message};
}

std::pair<bool, std::string> validateCartesianImpedanceParams(const msg::CartesianImpedanceParameters& params) {
  bool valid = true;
  std::string message;

  // Damping - unitless
  if (params.cartesian_damping.x < 0.1 || params.cartesian_damping.x > 1.0) {
    valid = false;
    message += "+Invalid DoF X damping";
  }
  if (params.cartesian_damping.y < 0.1 || params.cartesian_damping.y > 1.0) {
    valid = false;
    message += "+Invalid DoF Y damping";
  }
  if (params.cartesian_damping.z < 0.1 || params.cartesian_damping.z > 1.0) {
    valid = false;
    message += "+Invalid DoF Z damping";
  }
  if (params.cartesian_damping.a < 0.1 || params.cartesian_damping.a > 1.0) {
    valid = false;
    message += "+Invalid DoF A damping";
  }
  if (params.cartesian_damping.b < 0.1 || params.cartesian_damping.b > 1.0) {
    valid = false;
    message += "+Invalid DoF B damping";
  }
  if (params.cartesian_damping.c < 0.1 || params.cartesian_damping.c > 1.0) {
    valid = false;
    message += "+Invalid DoF C damping";
  }
  if (params.nullspace_damping < 0.3 || params.nullspace_damping > 1.0) {
    valid = false;
    message += "+Invalid nullspace damping";
  }

  // Stiffness - units N/m, Nm/rad, no idea for nullspace
  if (params.cartesian_stiffness.x < 0.0 || params.cartesian_stiffness.x > 5000.0) {
    valid = false;
    message += "+Invalid DoF X stiffness";
  }
  if (params.cartesian_stiffness.y < 0.0 || params.cartesian_stiffness.y > 5000.0) {
    valid = false;
    message += "+Invalid DoF Y stiffness";
  }
  if (params.cartesian_stiffness.z < 0.0 || params.cartesian_stiffness.z > 5000.0) {
    valid = false;
    message += "+Invalid DoF Z stiffness";
  }
  // TODO: original values set by Calder were < 0.1 and > 300.0 - documentation states < 0.0 and > 300.0; why was it
  // set to 0.1?
  if (params.cartesian_stiffness.a < 0.0 || params.cartesian_stiffness.a > 300.0) {
    valid = false;
    message += "+Invalid DoF A stiffness";
  }
  if (params.cartesian_stiffness.b < 0.0 || params.cartesian_stiffness.b > 300.0) {
    valid = false;
    message += "+Invalid DoF B stiffness";
  }
  if (params.cartesian_stiffness.c < 0.0 || params.cartesian_stiffness.c > 300.0) {
    valid = false;
    message += "+Invalid DoF C stiffness";
  }
  if (params.nullspace_stiffness < 0.0) {
    valid = false;
    message += "+Invalid nullspace stiffness";
  }

  return {valid, message};
}

std::pair<bool, std::string> validateCartesianControlModeLimits(const msg::CartesianControlModeLimits& params) {
  bool valid = true;
  std::string message;

  // Path deviation
  if (params.max_path_deviation.x <= 0.0) {
    valid = false;
    message += "+Invalid DoF X max path deviation";
  }
  if (params.max_path_deviation.y <= 0.0) {
    valid = false;
    message += "+Invalid DoF Y max path deviation";
  }
  if (params.max_path_deviation.z <= 0.0) {
    valid = false;
    message += "+Invalid DoF Z max path deviation";
  }
  if (params.max_path_deviation.a <= 0.0) {
    valid = false;
    message += "+Invalid DoF A max path deviation";
  }
  if (params.max_path_deviation.b <= 0.0) {
    valid = false;
    message += "+Invalid DoF B max path deviation";
  }
  if (params.max_path_deviation.c <= 0.0) {
    valid = false;
    message += "+Invalid DoF C max path deviation";
  }

  // Cartesian velocity
  if (params.max_cartesian_velocity.x <= 0.0) {
    valid = false;
    message += "+Invalid DoF X max cartesian velocity";
  }
  if (params.max_cartesian_velocity.y <= 0.0) {
    valid = false;
    message += "+Invalid DoF Y max cartesian velocity";
  }
  if (params.max_cartesian_velocity.z <= 0.0) {
    valid = false;
    message += "+Invalid DoF Z max cartesian velocity";
  }
  if (params.max_cartesian_velocity.a <= 0.0) {
    valid = false;
    message += "+Invalid DoF A max cartesian velocity";
  }
  if (params.max_cartesian_velocity.b <= 0.0) {
    valid = false;
    message += "+Invalid DoF B max cartesian velocity";
  }
  if (params.max_cartesian_velocity.c <= 0.0) {
    valid = false;
    message += "+Invalid DoF C max cartesian velocity";
  }

  // Cartesian force
  if (params.max_control_force.x <= 0.0) {
    valid = false;
    message += "+Invalid DoF X max control force";
  }
  if (params.max_control_force.y <= 0.0) {
    valid = false;
    message += "+Invalid DoF Y max control force";
  }
  if (params.max_control_force.z <= 0.0) {
    valid = false;
    message += "+Invalid DoF Z max control force";
  }
  if (params.max_control_force.a <= 0.0) {
    valid = false;
    message += "+Invalid DoF A max control force";
  }
  if (params.max_control_force.b <= 0.0) {
    valid = false;
    message += "+Invalid DoF B max control force";
  }
  if (params.max_control_force.c <= 0.0) {
    valid = false;
    message += "+Invalid DoF C max control force";
  }

  return {valid, message};
}

std::pair<bool, std::string> validateControlMode(const msg::ControlModeParameters& params) {
  bool valid = true;
  std::string message;

  // Check the control mode itself
  if (params.control_mode.mode != msg::ControlMode::JOINT_POSITION &&
      params.control_mode.mode != msg::ControlMode::JOINT_IMPEDANCE &&
      params.control_mode.mode != msg::ControlMode::CARTESIAN_POSE &&
      params.control_mode.mode != msg::ControlMode::CARTESIAN_IMPEDANCE) {
    valid = false;
    message += "+Invalid control mode";
  }

  // Check each part of the control mode
  const auto valid_joint_impedance_params = validateJointImpedanceParams(params.joint_impedance_params);
  const auto valid_cartesian_impedance_params = validateCartesianImpedanceParams(params.cartesian_impedance_params);
  const auto valid_cartesian_control_mode_limits =
      validateCartesianControlModeLimits(params.cartesian_control_mode_limits);
  const auto valid_joint_path_execution_params = validateJointPathExecutionParams(params.joint_path_execution_params);
  const auto valid_cartesian_path_execution_params =
      validateCartesianPathExecutionParams(params.cartesian_path_execution_params);

  // Aggregate the results
  valid &= valid_joint_impedance_params.first && valid_cartesian_impedance_params.first &&
           valid_cartesian_control_mode_limits.first && valid_joint_path_execution_params.first &&
           valid_cartesian_path_execution_params.first;

  message += valid_joint_impedance_params.second + valid_cartesian_impedance_params.second +
             valid_cartesian_control_mode_limits.second + valid_joint_path_execution_params.second +
             valid_cartesian_path_execution_params.second;

  return {valid, message};
}

std::pair<bool, std::string> validateCartesianPose(const geometry_msgs::msg::Pose& pose, const std::string& frame) {
  bool valid = true;
  std::string message;

  // Check to make sure the frame is correct
  if (frame != DEFAULT_CARTESIAN_CONTROL_FRAME) {
    valid = false;
    message += "Commanded cartesian pose has the wrong frame, " + frame + " given, " + DEFAULT_CARTESIAN_CONTROL_FRAME + " expected";
  }

  // Check to make sure the quaternion is well-formed
  {
    const double quat_squared_norm =
        (pose.orientation.w * pose.orientation.w) + (pose.orientation.x * pose.orientation.x) +
        (pose.orientation.y * pose.orientation.y) + (pose.orientation.z * pose.orientation.z);
    const double error = std::fabs(1.0 - quat_squared_norm);
    if (error > 1e-6) {
      valid = false;
      message +=
          "+Commanded cartesian pose quaternion is not normalized, squared norm = " + std::to_string(quat_squared_norm);
    }
  }
  return {valid, message};
}

std::pair<bool, std::string> validateMotionCommand(uint8_t const active_control_mode,
                                                   const victor_lcm_interface::motion_command& command) {
  const uint8_t command_motion_mode = command.control_mode.mode;
  if (active_control_mode != command_motion_mode) {
    return std::make_pair(false, std::string("Active control mode does not match commanded control mode"));
  }

  return {true, ""};
}

std::pair<bool, std::string> validateFingerCommand(const msg::Robotiq3FingerActuatorCommand& command) {
  bool valid = true;
  std::string message;
  if (command.position > 1.0 || command.position < 0.0) {
    valid = false;
    message += "+Invalid finger position";
  }

  if (command.force > 1.0 || command.force < 0.0) {
    valid = false;
    message += "+Invalid finger force";
  }

  if (command.speed > 1.0 || command.speed < 0.0) {
    valid = false;
    message += "+Invalid finger speed";
  }

  return {valid, message};
}

std::pair<bool, std::string> validateGripperCommand(const msg::Robotiq3FingerCommand& command) {
  const auto ac = validateFingerCommand(command.finger_a_command);
  const auto bc = validateFingerCommand(command.finger_b_command);
  const auto cc = validateFingerCommand(command.finger_c_command);
  const auto sc = validateFingerCommand(command.scissor_command);

  const bool valid = ac.first && bc.first && cc.first && sc.first;
  const std::string message = ac.second + bc.second + cc.second + sc.second;
  return {valid, message};
}

bool jointPathExecutionParamsIsDefault(const msg::JointPathExecutionParameters& params) {
  return (params.joint_relative_velocity == 0 && params.joint_relative_acceleration == 0 &&
          params.override_joint_acceleration == 0);
}

bool cartesianPathExecutionParamsIsDefault(const msg::CartesianPathExecutionParameters& params) {
  return (params.max_velocity.x == 0 && params.max_velocity.y == 0 && params.max_velocity.z == 0 &&
          params.max_velocity.a == 0 && params.max_velocity.b == 0 && params.max_velocity.c == 0 &&
          params.max_acceleration.x == 0 && params.max_acceleration.y == 0 && params.max_acceleration.z == 0 &&
          params.max_acceleration.a == 0 && params.max_acceleration.b == 0 && params.max_acceleration.c == 0 &&
          params.max_nullspace_velocity == 0 && params.max_nullspace_acceleration == 0);
}

bool jointImpedanceParamsIsDefault(const msg::JointImpedanceParameters& params) {
  return (params.joint_stiffness.joint_1 == 0 && params.joint_stiffness.joint_2 == 0 &&
          params.joint_stiffness.joint_3 == 0 && params.joint_stiffness.joint_4 == 0 &&
          params.joint_stiffness.joint_5 == 0 && params.joint_stiffness.joint_6 == 0 &&
          params.joint_stiffness.joint_7 == 0 && params.joint_damping.joint_1 == 0 &&
          params.joint_damping.joint_2 == 0 && params.joint_damping.joint_3 == 0 && params.joint_damping.joint_4 == 0 &&
          params.joint_damping.joint_5 == 0 && params.joint_damping.joint_6 == 0 && params.joint_damping.joint_7 == 0);
}

bool cartesianImpedanceParamsIsDefault(const msg::CartesianImpedanceParameters& params) {
  return (params.cartesian_stiffness.x == 0 && params.cartesian_stiffness.y == 0 && params.cartesian_stiffness.z == 0 &&
          params.cartesian_stiffness.a == 0 && params.cartesian_stiffness.b == 0 && params.cartesian_stiffness.c == 0 &&
          params.cartesian_damping.x == 0 && params.cartesian_damping.y == 0 && params.cartesian_damping.z == 0 &&
          params.cartesian_damping.a == 0 && params.cartesian_damping.b == 0 && params.cartesian_damping.c == 0 &&
          params.nullspace_stiffness == 0 && params.nullspace_damping == 0);
}

bool cartesianControlModeLimitsIsDefault(const msg::CartesianControlModeLimits& params) {
  return (params.max_path_deviation.x == 0 && params.max_path_deviation.y == 0 && params.max_path_deviation.z == 0 &&
          params.max_path_deviation.a == 0 && params.max_path_deviation.b == 0 && params.max_path_deviation.c == 0 &&
          params.max_cartesian_velocity.x == 0 && params.max_cartesian_velocity.y == 0 &&
          params.max_cartesian_velocity.z == 0 && params.max_cartesian_velocity.a == 0 &&
          params.max_cartesian_velocity.b == 0 && params.max_cartesian_velocity.c == 0 &&
          params.max_control_force.x == 0 && params.max_control_force.y == 0 && params.max_control_force.z == 0 &&
          params.max_control_force.a == 0 && params.max_control_force.b == 0 && params.max_control_force.c == 0 &&
          !params.stop_on_max_control_force);
}

msg::ControlModeParameters mergeControlModeParameters(const msg::ControlModeParameters& active_control_mode,
                                                      const msg::ControlModeParameters& new_control_mode) {
  /***************************************************************************************************************
  This function is a helper function for the callback function of setting a new control mode(setControlModeCallBack).
  It copies the parameters of the old control mode to the new one, and updates relevant parameters with theparameters
  of the new control mode.

  Parameters updated in each control mode:
  JOINT_POSITION: joint_path_execution_params
  CARTESIAN_POSE: cartesian_path_execution_params
  JOINT_IMPEDANCE: joint_impedance_params, joint_path_execution_params
  CARTESIAN_IMPEDANCE: cartesian_impedance_params, cartesian_control_mode_limits, cartesian_path_execution_params
  ***************************************************************************************************************/

  msg::ControlModeParameters merged_control_mode;
  // Copy the old over
  merged_control_mode.joint_path_execution_params = active_control_mode.joint_path_execution_params;
  merged_control_mode.joint_impedance_params = active_control_mode.joint_impedance_params;
  merged_control_mode.cartesian_impedance_params = active_control_mode.cartesian_impedance_params;
  merged_control_mode.cartesian_control_mode_limits = active_control_mode.cartesian_control_mode_limits;
  merged_control_mode.cartesian_path_execution_params = active_control_mode.cartesian_path_execution_params;
  // Copy manadatory members
  merged_control_mode.control_mode = new_control_mode.control_mode;
  // Copy mode-dependant members
  switch (new_control_mode.control_mode.mode) {
    case msg::ControlMode::JOINT_IMPEDANCE:
      merged_control_mode.joint_path_execution_params = new_control_mode.joint_path_execution_params;
      merged_control_mode.joint_impedance_params = new_control_mode.joint_impedance_params;

      if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params)) {
        RCLCPP_WARN(logger, "The cartesian impedance parameters are specified but ignored in JOINT_IMPEDANCE mode.");
      }
      if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits)) {
        RCLCPP_WARN(logger, "The cartesian control mode limits are specified but ignored in JOINT_IMPEDANCE mode.");
      }
      if (!cartesianPathExecutionParamsIsDefault(new_control_mode.cartesian_path_execution_params)) {
        RCLCPP_WARN(logger,
                    "The cartesian path execution parameters are specified but ignored in JOINT_IMPEDANCE mode.");
      }

      break;

    case msg::ControlMode::CARTESIAN_IMPEDANCE:
      merged_control_mode.cartesian_impedance_params = new_control_mode.cartesian_impedance_params;
      merged_control_mode.cartesian_control_mode_limits = new_control_mode.cartesian_control_mode_limits;
      merged_control_mode.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;

      if (!jointPathExecutionParamsIsDefault(new_control_mode.joint_path_execution_params)) {
        RCLCPP_WARN(logger,
                    "The joint path execution parameters are specified but ignored in CASRTESIAN_IMPEDANCE mode.");
      }
      if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params)) {
        RCLCPP_WARN(logger, "The joint impedance parameters are specified but ignored in CASRTESIAN_IMPEDANCE mode.");
      }

      break;

    case msg::ControlMode::JOINT_POSITION:
      // From the new
      merged_control_mode.joint_path_execution_params = new_control_mode.joint_path_execution_params;

      if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params)) {
        RCLCPP_WARN(logger, "The joint impedance parameters are specified but ignored in JOINT_POSITION mode.");
      }
      if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params)) {
        RCLCPP_WARN(logger, "The cartesian impedance parameters are specified but ignored in JOINT_POSITION mode.");
      }
      if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits)) {
        RCLCPP_WARN(logger, "The cartesian control mode limits are specified but ignored in JOINT_POSITION mode.");
      }
      if (!cartesianPathExecutionParamsIsDefault(new_control_mode.cartesian_path_execution_params)) {
        RCLCPP_WARN(logger,
                    "The cartesian path execution parameters are specified but ignored in JOINT_POSITION mode.");
      }

      break;

    case msg::ControlMode::CARTESIAN_POSE:
      // From the new
      merged_control_mode.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;

      if (!jointPathExecutionParamsIsDefault(new_control_mode.joint_path_execution_params)) {
        RCLCPP_WARN(logger, "The joint path execution parameters are specified but ignored in CARTESIAN_POSE mode.");
      }
      if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params)) {
        RCLCPP_WARN(logger, "The joint impedance parameters are specified but ignored in CARTESIAN_POSE mode.");
      }
      if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params)) {
        RCLCPP_WARN(logger, "The cartesian impedance parameters are specified but ignored in CARTESIAN_POSE mode.");
      }
      if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits)) {
        RCLCPP_WARN(logger, "The cartesian control mode limits are specified but ignored in CARTESIAN_POSE mode.");
      }

      break;

    default:
      RCLCPP_INFO_STREAM(logger, "Invalid control mode: " << new_control_mode.control_mode.mode << ".");
      assert(false);
  }

  return merged_control_mode;
}

};  // namespace victor_hardware
