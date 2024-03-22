#pragma once

#include <controller_interface/controller_interface.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <lcm/lcm-cpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <victor_hardware/kuka_control_mode_client.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

constexpr auto const DEFAULT_RELATIVE_VELOCITY = 0.1;

namespace victor_hardware {

class KukaJointPositionTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
 public:
  KukaJointPositionTrajectoryController() = default;

  controller_interface::CallbackReturn on_init() override;

 private:
  void updateControlModes();

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_;

  victor_lcm_interface::control_mode_parameters kuka_mode_params_ = default_control_mode_parameters();

  std::shared_ptr<KukaControlModeClient> left_control_mode_client_;
  std::shared_ptr<KukaControlModeClient> right_control_mode_client_;
};
}  // namespace victor_hardware
