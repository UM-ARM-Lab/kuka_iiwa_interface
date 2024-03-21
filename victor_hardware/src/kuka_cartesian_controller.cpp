#include <rclcpp/logger.hpp>
#include <victor_hardware/kuka_cartesian_controller.hpp>

static auto logger = rclcpp::get_logger("KukaCartesianController");

namespace victor_hardware {

controller_interface::CallbackReturn KukaCartesianController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_WARN(logger, "KukaCartesianController::on_deactivate");
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn KukaCartesianController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_WARN(logger, "KukaCartesianController::on_activate");
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn KukaCartesianController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_WARN(logger, "KukaCartesianController::on_configure");
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::return_type KukaCartesianController::update(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period) {
  auto clock = rclcpp::Clock();
  RCLCPP_WARN_THROTTLE(logger, clock, 500, "KukaCartesianController::update");
  return controller_interface::return_type::OK;
}
controller_interface::CallbackReturn KukaCartesianController::on_init() {
  RCLCPP_WARN(logger, "KukaCartesianController::on_init");
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration KukaCartesianController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration();
}
controller_interface::InterfaceConfiguration KukaCartesianController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration();
}
}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaCartesianController, controller_interface::ControllerInterface)
