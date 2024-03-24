#pragma once

#include <controller_interface/controller_interface.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <lcm/lcm-cpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <victor_hardware/kuka_control_mode_client.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

namespace victor_hardware {

class KukaJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
 public:
  KukaJointTrajectoryController() = default;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  int8_t control_mode_ = victor_lcm_interface::control_mode::JOINT_POSITION;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_;

  victor_lcm_interface::control_mode_parameters kuka_mode_params_ = default_control_mode_parameters();

  std::shared_ptr<KukaControlModeClientLifecycleNode> left_control_mode_client_;
  std::shared_ptr<KukaControlModeClientLifecycleNode> right_control_mode_client_;
};
}  // namespace victor_hardware
