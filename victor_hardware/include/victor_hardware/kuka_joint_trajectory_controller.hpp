#pragma once

#include <controller_interface/controller_interface.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <lcm/lcm-cpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <victor_hardware/control_mode_params_helper.hpp>
#include <victor_hardware/kuka_control_mode_client.hpp>
#include <victor_hardware/types.hpp>
#include <victor_hardware/validators.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

namespace victor_hardware {

class KukaJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
 public:
  KukaJointTrajectoryController() = default;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn on_init() override;

 protected:
  std::string control_mode_interface_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_;

  LCMPtr left_send_lcm_ptr_;
  LCMPtr right_send_lcm_ptr_;

  ControlModeParamsHelper::SharedPtr params_helper_;
};
}  // namespace victor_hardware
