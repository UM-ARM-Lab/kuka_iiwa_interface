#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>

namespace victor_sim_hardware {

// Type aliases for convenience
using ControlModeParameters = victor_hardware_interfaces::msg::ControlModeParameters;
using ControlModeParamsCallback = std::function<void(const ControlModeParameters&)>;

// Use control mode constants from the message interface
namespace control_mode {
constexpr int8_t JOINT_POSITION = 1;
constexpr int8_t JOINT_IMPEDANCE = 2;
constexpr int8_t CARTESIAN_POSE = 3;
constexpr int8_t CARTESIAN_IMPEDANCE = 4;
}

// Default parameter constants
constexpr double DEFAULT_JOINT_RELATIVE_VELOCITY = 0.1;
constexpr double DEFAULT_JOINT_RELATIVE_ACCELERATION = 0.1;
constexpr bool DEFAULT_OVERRIDE_JOINT_ACCELERATION = false;

constexpr double DEFAULT_JOINT_DAMPING = 0.7;
constexpr double DEFAULT_JOINT1_STIFFNESS = 3000.0;
constexpr double DEFAULT_JOINT2_STIFFNESS = 3000.0;
constexpr double DEFAULT_JOINT3_STIFFNESS = 3000.0;
constexpr double DEFAULT_JOINT4_STIFFNESS = 300.0;
constexpr double DEFAULT_JOINT5_STIFFNESS = 300.0;
constexpr double DEFAULT_JOINT6_STIFFNESS = 300.0;
constexpr double DEFAULT_JOINT7_STIFFNESS = 300.0;

constexpr double DEFAULT_MAX_LIN_VELOCITY = 0.2;
constexpr double DEFAULT_MAX_ROT_VELOCITY = 0.5;
constexpr double DEFAULT_MAX_NULLSPACE_VELOCITY = 0.2;
constexpr double DEFAULT_MAX_LIN_ACCELERATION = 1.0;
constexpr double DEFAULT_MAX_ROT_ACCELERATION = 2.0;
constexpr double DEFAULT_MAX_NULLSPACE_ACCELERATION = 1.0;

constexpr double DEFAULT_CARTESIAN_DAMPING = 0.7;
constexpr double DEFAULT_NULLSPACE_DAMPING = 0.7;
constexpr double DEFAULT_CARTESIAN_STIFFNESS = 3000.0;
constexpr double DEFAULT_ROT_STIFFNESS = 300.0;
constexpr double DEFAULT_NULLSPACE_STIFFNESS = 100.0;

constexpr double DEFAULT_MAX_PATH_DEVIATION = 10.0;
constexpr double DEFAULT_MAX_CARTESIAN_LIN_VELOCITY = 0.2;
constexpr double DEFAULT_MAX_CARTESIAN_ROT_VELOCITY = 0.5;
constexpr double DEFAULT_MAX_CONTROL_FORCE = 100.0;
constexpr bool DEFAULT_STOP_ON_MAX_CONTROL_FORCE = true;

/**
 * @brief Simulates control mode client functionality for the simulator hardware interface.
 * 
 * This class provides the same interface as the real hardware's KukaControlModeClient
 * but adapted for the simulator environment. It manages control mode parameters and
 * publishes them via callback when control modes are switched.
 */
class SimControlModeClient {
public:
    /**
     * @brief Constructor
     * @param node Shared pointer to the ROS node
     * @param callback Callback function to call when control mode parameters are updated
     */
    SimControlModeClient(std::shared_ptr<rclcpp::Node> node, ControlModeParamsCallback callback);

    /**
     * @brief Update the control mode and trigger parameter publishing
     * @param mode The new control mode to switch to
     * @return true if successful, false otherwise
     */
    bool updateControlMode(int8_t mode);

    /**
     * @brief Get the current control mode parameters
     * @return Current control mode parameters
     */
    ControlModeParameters getControlMode() const;

private:
    /**
     * @brief Initialize default control mode parameters
     */
    void initializeDefaultParams();

    /**
     * @brief Create default control mode parameters with safe values
     * @return Default control mode parameters
     */
    ControlModeParameters createDefaultParams() const;

    std::shared_ptr<rclcpp::Node> node_;
    ControlModeParamsCallback callback_;
    ControlModeParameters current_params_;
    rclcpp::Logger logger_;
};

}  // namespace victor_sim_hardware
