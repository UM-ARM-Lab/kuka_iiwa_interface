#pragma once

#include <rclcpp/rclcpp.hpp>
#include <victor_hardware/kuka_cartesian_controller.hpp>

namespace victor_hardware {

class KukaLocalCartesianController : public KukaCartesianController {
 public:
  controller_interface::CallbackReturn on_init() override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 protected:
};
}  // namespace victor_hardware
