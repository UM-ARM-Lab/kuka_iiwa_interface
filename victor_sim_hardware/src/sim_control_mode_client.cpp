#include "victor_sim_hardware/sim_control_mode_client.hpp"

namespace victor_sim_hardware {

SimControlModeClient::SimControlModeClient(std::shared_ptr<rclcpp::Node> node,
                                         ControlModeParamsCallback callback)
    : node_(node), callback_(callback), logger_(rclcpp::get_logger("SimControlModeClient")) {
    initializeDefaultParams();
}

bool SimControlModeClient::updateControlMode(int8_t mode) {
    if (mode < control_mode::JOINT_POSITION || mode > control_mode::CARTESIAN_IMPEDANCE) {
        RCLCPP_ERROR(logger_, "Invalid control mode: %d", mode);
        return false;
    }

    current_params_.control_mode.mode = mode;
    
    // Call the callback with updated parameters
    if (callback_) {
        callback_(current_params_);
    }

    RCLCPP_INFO(logger_, "Updated control mode to: %d", mode);
    return true;
}

ControlModeParameters SimControlModeClient::getControlMode() const {
    return current_params_;
}

void SimControlModeClient::initializeDefaultParams() {
    current_params_ = createDefaultParams();
}

ControlModeParameters SimControlModeClient::createDefaultParams() const {
    ControlModeParameters params{};
    
    // Initialize with default values from constants
    params.control_mode.mode = control_mode::JOINT_POSITION;
    
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

}  // namespace victor_sim_hardware
