#pragma once

#include <controller_interface/controller_interface.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

constexpr auto const DEFAULT_RELATIVE_VELOCITY= 0.1;

namespace victor_hardware {

class KukaJointPositionTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
 public:
  KukaJointPositionTrajectoryController() = default;

  controller_interface::CallbackReturn on_init() override;

 private:
  void sendParamsLCM();

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> handles_;

  victor_lcm_interface::control_mode_parameters kuka_mode_params_;
};
}  // namespace victor_hardware
