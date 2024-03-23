#pragma once

#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <victor_hardware/lcm_listener.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

template <typename NodeType>
class KukaControlModeClient {
 public:
  KukaControlModeClient(std::shared_ptr<NodeType> node, std::string const& recv_provider,
                        std::string const& send_provider)
      : node_(node) {
    recv_lcm_ptr_ = std::make_shared<lcm::LCM>(recv_provider);
    send_lcm_ptr_ = std::make_shared<lcm::LCM>(send_provider);

    control_mode_listener_ = std::make_unique<LcmListener<victor_lcm_interface::control_mode_parameters>>(
        recv_lcm_ptr_, DEFAULT_CONTROL_MODE_STATUS_CHANNEL,
        [&](victor_lcm_interface::control_mode_parameters const& msg) {});  // nothing to do here
  };

  [[nodiscard]] bool updateControlMode(victor_lcm_interface::control_mode_parameters const& new_control_mode) {
    send_lcm_ptr_->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &new_control_mode);

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
      if (current_control_mode == new_control_mode.control_mode.mode) {
        RCLCPP_INFO(node_->get_logger(), "Successfully changed control mode");
        return true;
      }
      if (std::chrono::steady_clock::now() - start_time > DEFAULT_SET_CONTROL_MODE_TIMEOUT) {
        RCLCPP_WARN(node_->get_logger(), "Control mode change timed out");
        return false;
      }
    }
  };

  [[nodiscard]] victor_lcm_interface::control_mode_parameters getControlMode() const {
    while (true) {
      recv_lcm_ptr_->handleTimeout(1000);
      if (!control_mode_listener_->hasLatestMessage()) {
        continue;
      }
      auto const& control_mode_params = control_mode_listener_->getLatestMessage();
      return control_mode_params;
    }
  };

 private:
  std::shared_ptr<NodeType> node_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
  std::shared_ptr<lcm::LCM> send_lcm_ptr_;

  // listener
  std::unique_ptr<LcmListener<victor_lcm_interface::control_mode_parameters>> control_mode_listener_;
};

using KukaControlModeClientLifecycleNode = KukaControlModeClient<rclcpp_lifecycle::LifecycleNode>;
using KukaControlModeClientNode = KukaControlModeClient<rclcpp::Node>;
