#pragma once

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/control_mode_params_helper.hpp>
#include <victor_hardware/kuka_control_mode_client.hpp>
#include <victor_hardware/types.hpp>
#include <victor_hardware/validators.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

namespace victor_hardware {

class KukaCartesianController : public controller_interface::ControllerInterface {
 public:
  KukaCartesianController() = default;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 protected:
  std::string side_name_;
  std::string arm_name_;
  std::string control_mode_interface_;

  std::optional<geometry_msgs::msg::Pose> latest_cmd_msg_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_sub_;

  LCMPtr send_lcm_ptr_;

  ControlModeParamsHelper::SharedPtr params_helper_;
};
}  // namespace victor_hardware
