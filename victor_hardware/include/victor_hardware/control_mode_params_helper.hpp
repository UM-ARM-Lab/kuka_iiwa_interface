#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_control_mode_client.hpp>
#include <victor_hardware/lcm_ostream_operators.hpp>
#include <victor_hardware/types.hpp>
#include <victor_hardware/validators.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

namespace victor_hardware {

class ControlModeParamsHelper {
 public:
  using SharedPtr = std::shared_ptr<ControlModeParamsHelper>;

  explicit ControlModeParamsHelper(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::vector<LCMPtr> send_lcm_ptrs, int8_t const mode)
      : node_(node), send_lcm_ptrs_(send_lcm_ptrs) {
      kuka_mode_params_.control_mode.mode = mode;
      }

  ErrorType updateControlModeParams() {
    // only do this if the controller is active!
    if (node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return {true, ""};
    }

    auto const &validate_mode_result = validateControlMode(kuka_mode_params_);
    if (!validate_mode_result.first) {
      return validate_mode_result;
    }

    RCLCPP_INFO(node_->get_logger(), "Updating control mode params: ");
    RCLCPP_INFO_STREAM(node_->get_logger(), kuka_mode_params_);

    // Send it a bunch of times to make sure it gets there, and to stall to let the change take place...
    // Since the HW IF will be reading and sending the new params
    for (auto i{0}; i < 10; ++i) {
      for (auto const &send_lcm_ptr : send_lcm_ptrs_) {
        send_lcm_ptr->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &kuka_mode_params_);
      }

      usleep(1000);
    }

    return {true, ""};
  }

  victor_lcm_interface::control_mode_parameters &params() { return kuka_mode_params_; }

 private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  victor_lcm_interface::control_mode_parameters kuka_mode_params_ = default_control_mode_parameters();
  LCMPtrs send_lcm_ptrs_;
};

}  // namespace victor_hardware