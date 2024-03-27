#pragma once

#include <position_controllers/joint_group_position_controller.hpp>
#include <victor_hardware/control_mode_params_helper.hpp>
#include <victor_hardware/types.hpp>

namespace victor_hardware {

class KukaJointGroupPositionController : public position_controllers::JointGroupPositionController {
 public:
  KukaJointGroupPositionController() = default;

  controller_interface::CallbackReturn on_init() override;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

 protected:
  // The command mode interface
  std::string control_mode_interface_;
  std::string side_name_;

  LCMPtr send_lcm_ptr_;

  ControlModeParamsHelper::SharedPtr params_helper_;
};

}  // namespace victor_hardware
