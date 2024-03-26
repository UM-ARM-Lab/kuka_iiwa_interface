#include <cmath>
#include <optional>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/validators.hpp>

namespace victor_hardware {

ErrorType validateJointPathExecutionParams(const victor_lcm_interface::joint_path_execution_parameters &params) {
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

ErrorType validateCartesianPathExecutionParams(
    const victor_lcm_interface::cartesian_path_execution_parameters &params) {
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

ErrorType validateJointImpedanceParams(const victor_lcm_interface::joint_impedance_parameters &params) {
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

ErrorType validateCartesianImpedanceParams(const victor_lcm_interface::cartesian_impedance_parameters &params) {
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

ErrorType validateCartesianControlModeLimits(const victor_lcm_interface::cartesian_control_mode_limits &params) {
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

ErrorType validateControlMode(const victor_lcm_interface::control_mode_parameters &params) {
  bool valid = true;
  std::string message;

  // Check the control mode itself
  if (params.control_mode.mode != victor_lcm_interface::control_mode::JOINT_POSITION &&
      params.control_mode.mode != victor_lcm_interface::control_mode::JOINT_IMPEDANCE &&
      params.control_mode.mode != victor_lcm_interface::control_mode::CARTESIAN_POSE &&
      params.control_mode.mode != victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE) {
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

ErrorType validateCartesianPose(const geometry_msgs::msg::Pose &pose, const std::string &frame) {
  bool valid = true;
  std::string message;

  // Check to make sure the frame is correct
  if (frame != DEFAULT_CARTESIAN_CONTROL_FRAME) {
    valid = false;
    message += "Commanded cartesian pose has the wrong frame, " + frame + " given, " + DEFAULT_CARTESIAN_CONTROL_FRAME +
               " expected";
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

ErrorType validateMotionCommand(const victor_lcm_interface::motion_command &command) {
  // Check for very likely to be invalid commands
  if (command.control_mode.mode == victor_lcm_interface::control_mode::CARTESIAN_POSE ||
      command.control_mode.mode == victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE) {
    auto const &zero_pose = command.cartesian_pose.xt == 0.0 && command.cartesian_pose.yt == 0.0 &&
                            command.cartesian_pose.zt == 0.0 && command.cartesian_pose.xr == 0.0 &&
                            command.cartesian_pose.yr == 0.0 && command.cartesian_pose.zr == 0.0 &&
                            command.cartesian_pose.wr == 0.0;
    auto const &nan_pose = std::isnan(command.cartesian_pose.xt) || std::isnan(command.cartesian_pose.yt) ||
                           std::isnan(command.cartesian_pose.zt) || std::isnan(command.cartesian_pose.xr) ||
                           std::isnan(command.cartesian_pose.yr) || std::isnan(command.cartesian_pose.zr) ||
                           std::isnan(command.cartesian_pose.wr);
    if (zero_pose) {
      return {false,
              "Commanded cartesian pose is all zeros! That would cause the end-effector to crash into the base"
              " Perhaps the pose is uninitialized?"};
    }
    if (nan_pose) {
      return {false, "Commanded cartesian pose contains NaN"};
    }
  } else if (command.control_mode.mode == victor_lcm_interface::control_mode::JOINT_POSITION ||
             command.control_mode.mode == victor_lcm_interface::control_mode::JOINT_IMPEDANCE) {
    auto const &nan_joint = std::isnan(command.joint_position.joint_1) || std::isnan(command.joint_position.joint_2) ||
                            std::isnan(command.joint_position.joint_3) || std::isnan(command.joint_position.joint_4) ||
                            std::isnan(command.joint_position.joint_5) || std::isnan(command.joint_position.joint_6) ||
                            std::isnan(command.joint_position.joint_7);
    if (nan_joint) {
      return {false, "Commanded joint position contains NaN"};
    }
  } else {
    return {false, "Invalid control mode"};
  }

  return {true, ""};
}

ErrorType validateFingerCommand(const msg::Robotiq3FingerActuatorCommand &command) {
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

bool operator==(victor_lcm_interface::control_mode_parameters const &p1,
                victor_lcm_interface::control_mode_parameters const &p2) {
  return p1.control_mode == p2.control_mode && p1.joint_impedance_params == p2.joint_impedance_params &&
         p1.joint_path_execution_params == p2.joint_path_execution_params &&
         p1.cartesian_impedance_params == p2.cartesian_impedance_params &&
         p1.cartesian_control_mode_limits == p2.cartesian_control_mode_limits &&
         p1.cartesian_path_execution_params == p2.cartesian_path_execution_params;
}

bool operator==(victor_lcm_interface::control_mode const &p1, victor_lcm_interface::control_mode const &p2) {
  return p1.mode == p2.mode;
}

bool operator==(victor_lcm_interface::joint_impedance_parameters const &p1,
                victor_lcm_interface::joint_impedance_parameters const &p2) {
  return p1.joint_damping == p2.joint_damping && p1.joint_stiffness == p2.joint_stiffness;
}

bool operator==(victor_lcm_interface::cartesian_impedance_parameters const &p1,
                victor_lcm_interface::cartesian_impedance_parameters const &p2) {
  return p1.cartesian_damping == p2.cartesian_damping && p1.cartesian_stiffness == p2.cartesian_stiffness &&
         p1.nullspace_damping == p2.nullspace_damping && p1.nullspace_stiffness == p2.nullspace_stiffness;
}

bool operator==(victor_lcm_interface::cartesian_control_mode_limits const &p1,
                victor_lcm_interface::cartesian_control_mode_limits const &p2) {
  return p1.max_path_deviation == p2.max_path_deviation && p1.max_cartesian_velocity == p2.max_cartesian_velocity &&
         p1.max_control_force == p2.max_control_force && p1.stop_on_max_control_force == p2.stop_on_max_control_force;
}

bool operator==(victor_lcm_interface::cartesian_path_execution_parameters const &p1,
                victor_lcm_interface::cartesian_path_execution_parameters const &p2) {
  return p1.max_velocity == p2.max_velocity && p1.max_acceleration == p2.max_acceleration &&
         p1.max_nullspace_velocity == p2.max_nullspace_velocity &&
         p1.max_nullspace_acceleration == p2.max_nullspace_acceleration;
}

bool operator==(victor_lcm_interface::joint_path_execution_parameters const &p1,
                victor_lcm_interface::joint_path_execution_parameters const &p2) {
  return p1.joint_relative_velocity == p2.joint_relative_velocity &&
         p1.joint_relative_acceleration == p2.joint_relative_acceleration &&
         p1.override_joint_acceleration == p2.override_joint_acceleration;
}

bool operator==(victor_lcm_interface::joint_value_quantity const &p1,
                victor_lcm_interface::joint_value_quantity const &p2) {
  return p1.joint_1 == p2.joint_1 && p1.joint_2 == p2.joint_2 && p1.joint_3 == p2.joint_3 && p1.joint_4 == p2.joint_4 &&
         p1.joint_5 == p2.joint_5 && p1.joint_6 == p2.joint_6 && p1.joint_7 == p2.joint_7;
}

bool operator==(victor_lcm_interface::cartesian_pose const &p1, victor_lcm_interface::cartesian_pose const &p2) {
  return p1.xt == p2.xt && p1.yt == p2.yt && p1.zt == p2.zt && p1.wr == p2.wr && p1.xr == p2.xr && p1.yr == p2.yr &&
         p1.zr == p2.zr;
}

bool operator==(victor_lcm_interface::cartesian_value_quantity const &p1,
                victor_lcm_interface::cartesian_value_quantity const &p2) {
  return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z && p1.a == p2.a && p1.b == p2.b && p1.c == p2.c;
}

};  // namespace victor_hardware
