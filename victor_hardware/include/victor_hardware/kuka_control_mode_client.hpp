#pragma once

#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/lcm_listener.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

template <typename NodeType>
class KukaControlModeClient {
 public:
  /** If this is the only object that's going to be using LCM to set/get control mode, then use this constructor **/
  KukaControlModeClient(std::shared_ptr<NodeType> node, std::string const& recv_provider,
                        std::string const& send_provider)
      : node_(node) {
    recv_lcm_ptr_ = std::make_shared<lcm::LCM>(recv_provider);
    send_lcm_ptr_ = std::make_shared<lcm::LCM>(send_provider);

    control_mode_listener_ = std::make_unique<LcmListener<victor_lcm_interface::control_mode_parameters>>(
        recv_lcm_ptr_, DEFAULT_CONTROL_MODE_STATUS_CHANNEL,
        [&](victor_lcm_interface::control_mode_parameters const& msg) {});  // nothing to do here
  };

  /** Only one process/thread can subscribe to a LCM channel, so use this constructor if you're already doing LCM **/
  KukaControlModeClient(std::shared_ptr<NodeType> node, std::shared_ptr<lcm::LCM> const &recv_provider,
                        std::shared_ptr<lcm::LCM> const &send_provider)
      : node_(node), recv_lcm_ptr_(recv_provider), send_lcm_ptr_(send_provider) {
    control_mode_listener_ = std::make_unique<LcmListener<victor_lcm_interface::control_mode_parameters>>(
        recv_lcm_ptr_, DEFAULT_CONTROL_MODE_STATUS_CHANNEL,
        [&](victor_lcm_interface::control_mode_parameters const& msg) {});  // nothing to do here
  };

  /** ONLY updates the control mode, not the control mode parameters */
  [[nodiscard]] bool updateControlMode(int8_t const& new_control_mode) {
    auto active_control_mode_params = getControlMode();
    active_control_mode_params.control_mode.mode = new_control_mode;
    return updateAndWaitForConfirmation(active_control_mode_params);
  }

  /** does NOT update the control mode itself **/
  [[nodiscard]] bool updateControlModeParameters(
      victor_lcm_interface::control_mode_parameters const& new_control_mode) {
    auto active_control_mode_params = getControlMode();

    active_control_mode_params.cartesian_control_mode_limits = new_control_mode.cartesian_control_mode_limits;
    active_control_mode_params.cartesian_impedance_params = new_control_mode.cartesian_impedance_params;
    active_control_mode_params.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;
    active_control_mode_params.joint_impedance_params = new_control_mode.joint_impedance_params;
    active_control_mode_params.joint_path_execution_params = new_control_mode.joint_path_execution_params;

    send_lcm_ptr_->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &new_control_mode);
    return updateAndWaitForConfirmation(active_control_mode_params);
  };

  [[nodiscard]] victor_lcm_interface::control_mode_parameters getControlMode() const {
    while (true) {
//      RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting to get control mode...");
      recv_lcm_ptr_->handleTimeout(1000);
      if (!control_mode_listener_->hasLatestMessage()) {
        continue;
      }
      auto const& control_mode_params = control_mode_listener_->getLatestMessage();
      return control_mode_params;
    }
  };

 private:
  bool updateAndWaitForConfirmation(const victor_lcm_interface::control_mode_parameters& active_control_mode_params) {
    send_lcm_ptr_->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &active_control_mode_params);

    // Check to see if the control mode has changed, and if it hasn't after a certain amount of time,
    // log an error message and return false
    auto const start_time = std::chrono::steady_clock::now();
    while (true) {
      recv_lcm_ptr_->handleTimeout(1000);
      if (!control_mode_listener_->hasLatestMessage()) {
        continue;
      }
      auto const& control_mode_params = control_mode_listener_->getLatestMessage();
      auto const& current_control_mode = control_mode_params.control_mode.mode;
      if (current_control_mode == active_control_mode_params.control_mode.mode) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Successfully changed control mode to " << std::to_string((int)current_control_mode));
        return true;
      }
      if (std::chrono::steady_clock::now() - start_time > DEFAULT_SET_CONTROL_MODE_TIMEOUT) {
        RCLCPP_WARN(node_->get_logger(), "Control mode change timed out");
        return false;
      }
    }
  };

  std::shared_ptr<NodeType> node_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
  std::shared_ptr<lcm::LCM> send_lcm_ptr_;

  // listener
  std::unique_ptr<LcmListener<victor_lcm_interface::control_mode_parameters>> control_mode_listener_;
};

using KukaControlModeClientLifecycleNode = KukaControlModeClient<rclcpp_lifecycle::LifecycleNode>;
using KukaControlModeClientNode = KukaControlModeClient<rclcpp::Node>;
