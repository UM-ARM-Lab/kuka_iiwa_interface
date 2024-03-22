#pragma once

#include <controller_interface/controller_interface.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>

namespace victor_hardware {

class KukaJointPositionTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
 public:
  KukaJointPositionTrajectoryController() = default;

  controller_interface::CallbackReturn on_init() override;

 private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
};
}  // namespace victor_hardware
