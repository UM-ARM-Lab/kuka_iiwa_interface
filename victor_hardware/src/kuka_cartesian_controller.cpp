#include <rclcpp/logger.hpp>
#include <string>
#include <vector>
#include <victor_hardware/kuka_cartesian_controller.hpp>

static auto logger = rclcpp::get_logger("KukaCartesianController");

namespace victor_hardware {

controller_interface::CallbackReturn KukaCartesianController::on_init() {
  auto node = get_node();
  RCLCPP_WARN_STREAM(logger, "node name: " << node->get_name());

//  node->declare_parameter<std::string>("side");

  RCLCPP_WARN(logger, "KukaCartesianController::on_init");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration KukaCartesianController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;

    std::string side_name = "left";
//  std::string side_name = get_node()->get_parameter("side").as_string();
//  RCLCPP_WARN_STREAM(logger, "KukaCartesianController::command_interface_configuration: " << side_name);

  // Claim the cartesian command interface only
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(side_name + "/cartesian_pos/xt");
  config.names.push_back(side_name + "/cartesian_pos/yt");
  config.names.push_back(side_name + "/cartesian_pos/zt");
  config.names.push_back(side_name + "/cartesian_pos/wr");
  config.names.push_back(side_name + "/cartesian_pos/xr");
  config.names.push_back(side_name + "/cartesian_pos/yr");
  config.names.push_back(side_name + "/cartesian_pos/zr");

  return config;
}

controller_interface::InterfaceConfiguration KukaCartesianController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration(controller_interface::interface_configuration_type::NONE);
}

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
}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaCartesianController, controller_interface::ControllerInterface)
