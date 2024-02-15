#include <victor_hardware/victor_lcm_bridge_utils.hpp>
#include <rclcpp/logging.hpp>

auto const logger = rclcpp::get_logger("victor_lcm_bridge_utils");

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers to test if two messages are equivalent
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace victor_hardware {

bool jvqEqual(const msg::JointValueQuantity& jvq1, const msg::JointValueQuantity& jvq2) {
  if (jvq1.joint_1 != jvq2.joint_1) {
    return false;
  } else if (jvq1.joint_2 != jvq2.joint_2) {
    return false;
  } else if (jvq1.joint_3 != jvq2.joint_3) {
    return false;
  } else if (jvq1.joint_4 != jvq2.joint_4) {
    return false;
  } else if (jvq1.joint_5 != jvq2.joint_5) {
    return false;
  } else if (jvq1.joint_6 != jvq2.joint_6) {
    return false;
  } else if (jvq1.joint_7 != jvq2.joint_7) {
    return false;
  } else {
    return true;
  }
}

bool cvqEqual(const msg::CartesianValueQuantity& cvq1, const msg::CartesianValueQuantity& cvq2) {
  if (cvq1.x != cvq2.x) {
    return false;
  } else if (cvq1.y != cvq2.y) {
    return false;
  } else if (cvq1.z != cvq2.z) {
    return false;
  } else if (cvq1.a != cvq2.a) {
    return false;
  } else if (cvq1.b != cvq2.b) {
    return false;
  } else if (cvq1.c != cvq2.c) {
    return false;
  } else {
    return true;
  }
}

bool jointPexpEqual(const msg::JointPathExecutionParameters& pexp1, const msg::JointPathExecutionParameters& pexp2) {
  if (pexp1.joint_relative_acceleration != pexp2.joint_relative_acceleration) {
    return false;
  } else if (pexp1.joint_relative_velocity != pexp2.joint_relative_velocity) {
    return false;
  } else if (pexp1.override_joint_acceleration != pexp2.override_joint_acceleration) {
    return false;
  } else {
    return true;
  }
}

bool cartesianPexpEqual(const msg::CartesianPathExecutionParameters& pexp1,
                        const msg::CartesianPathExecutionParameters& pexp2) {
  if (!cvqEqual(pexp1.max_velocity, pexp2.max_velocity)) {
    return false;
  } else if (!cvqEqual(pexp1.max_acceleration, pexp2.max_acceleration)) {
    return false;
  } else if (pexp1.max_nullspace_velocity != pexp2.max_nullspace_velocity) {
    return false;
  } else if (pexp1.max_nullspace_acceleration != pexp2.max_nullspace_acceleration) {
    return false;
  } else {
    return true;
  }
}

bool controlModeParamsEqual(const msg::ControlModeParameters& params1, const msg::ControlModeParameters& params2) {
  // Control mode parameter
  const bool cm_equal = (params1.control_mode.mode == params2.control_mode.mode);

  // Path Execution mode parameters
  const bool jpexp_equal = jointPexpEqual(params1.joint_path_execution_params, params2.joint_path_execution_params);
  const bool cpexp_equal =
      cartesianPexpEqual(params1.cartesian_path_execution_params, params2.cartesian_path_execution_params);

  // Joint Impedance mode parameters
  const bool jd_equal =
      jvqEqual(params1.joint_impedance_params.joint_damping, params2.joint_impedance_params.joint_damping);
  const bool js_equal =
      jvqEqual(params1.joint_impedance_params.joint_stiffness, params2.joint_impedance_params.joint_stiffness);

  // Cartesian Impedance mode parameters
  const bool cd_equal = cvqEqual(params1.cartesian_impedance_params.cartesian_damping,
                                 params2.cartesian_impedance_params.cartesian_damping);
  const bool nd_equal =
      (params1.cartesian_impedance_params.nullspace_damping == params2.cartesian_impedance_params.nullspace_damping);
  const bool cs_equal = cvqEqual(params1.cartesian_impedance_params.cartesian_stiffness,
                                 params2.cartesian_impedance_params.cartesian_stiffness);
  const bool ns_equal = (params1.cartesian_impedance_params.nullspace_stiffness ==
                         params2.cartesian_impedance_params.nullspace_stiffness);

  // Cartesian control mode limits parameters
  const bool mpd_equal = cvqEqual(params1.cartesian_control_mode_limits.max_path_deviation,
                                  params2.cartesian_control_mode_limits.max_path_deviation);
  const bool mcv_equal = cvqEqual(params1.cartesian_control_mode_limits.max_cartesian_velocity,
                                  params2.cartesian_control_mode_limits.max_cartesian_velocity);
  const bool mcf_equal = cvqEqual(params1.cartesian_control_mode_limits.max_control_force,
                                  params2.cartesian_control_mode_limits.max_control_force);
  const bool smcf_equal = (params1.cartesian_control_mode_limits.stop_on_max_control_force ==
                           params2.cartesian_control_mode_limits.stop_on_max_control_force);

  if (cm_equal && jpexp_equal && cpexp_equal && jd_equal && js_equal && cd_equal && nd_equal && cs_equal && ns_equal &&
      mpd_equal && mcv_equal && mcf_equal && smcf_equal) {
    return true;
  } else {
    return false;
  }
}

static bool jointPathExecutionParamsIsDefault(const msg::JointPathExecutionParameters& params) {
  return (params.joint_relative_velocity == 0 && params.joint_relative_acceleration == 0 &&
          params.override_joint_acceleration == 0);
}

static bool cartesianPathExecutionParamsIsDefault(const msg::CartesianPathExecutionParameters& params) {
  return (params.max_velocity.x == 0 && params.max_velocity.y == 0 && params.max_velocity.z == 0 &&
          params.max_velocity.a == 0 && params.max_velocity.b == 0 && params.max_velocity.c == 0 &&
          params.max_acceleration.x == 0 && params.max_acceleration.y == 0 && params.max_acceleration.z == 0 &&
          params.max_acceleration.a == 0 && params.max_acceleration.b == 0 && params.max_acceleration.c == 0 &&
          params.max_nullspace_velocity == 0 && params.max_nullspace_acceleration == 0);
}

static bool jointImpedanceParamsIsDefault(const msg::JointImpedanceParameters& params) {
  return (params.joint_stiffness.joint_1 == 0 && params.joint_stiffness.joint_2 == 0 &&
          params.joint_stiffness.joint_3 == 0 && params.joint_stiffness.joint_4 == 0 &&
          params.joint_stiffness.joint_5 == 0 && params.joint_stiffness.joint_6 == 0 &&
          params.joint_stiffness.joint_7 == 0 && params.joint_damping.joint_1 == 0 &&
          params.joint_damping.joint_2 == 0 && params.joint_damping.joint_3 == 0 && params.joint_damping.joint_4 == 0 &&
          params.joint_damping.joint_5 == 0 && params.joint_damping.joint_6 == 0 && params.joint_damping.joint_7 == 0);
}

static bool cartesianImpedanceParamsIsDefault(const msg::CartesianImpedanceParameters& params) {
  return (params.cartesian_stiffness.x == 0 && params.cartesian_stiffness.y == 0 && params.cartesian_stiffness.z == 0 &&
          params.cartesian_stiffness.a == 0 && params.cartesian_stiffness.b == 0 && params.cartesian_stiffness.c == 0 &&
          params.cartesian_damping.x == 0 && params.cartesian_damping.y == 0 && params.cartesian_damping.z == 0 &&
          params.cartesian_damping.a == 0 && params.cartesian_damping.b == 0 && params.cartesian_damping.c == 0 &&
          params.nullspace_stiffness == 0 && params.nullspace_damping == 0);
}

static bool cartesianControlModeLimitsIsDefault(const msg::CartesianControlModeLimits& params) {
  return (params.max_path_deviation.x == 0 && params.max_path_deviation.y == 0 && params.max_path_deviation.z == 0 &&
          params.max_path_deviation.a == 0 && params.max_path_deviation.b == 0 && params.max_path_deviation.c == 0 &&
          params.max_cartesian_velocity.x == 0 && params.max_cartesian_velocity.y == 0 &&
          params.max_cartesian_velocity.z == 0 && params.max_cartesian_velocity.a == 0 &&
          params.max_cartesian_velocity.b == 0 && params.max_cartesian_velocity.c == 0 &&
          params.max_control_force.x == 0 && params.max_control_force.y == 0 && params.max_control_force.z == 0 &&
          params.max_control_force.a == 0 && params.max_control_force.b == 0 && params.max_control_force.c == 0 &&
          params.stop_on_max_control_force == false);
}

static msg::ControlModeParameters mergeControlModeParameters(const msg::ControlModeParameters& active_control_mode,
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

}