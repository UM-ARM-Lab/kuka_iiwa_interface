#include <lifecycle_msgs/msg/state.hpp>
#include <string>
#include <vector>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_cartesian_controller.hpp>
#include <victor_hardware/lcm_ostream_operators.hpp>
#include <victor_hardware/validators.hpp>

namespace victor_hardware {

controller_interface::CallbackReturn KukaCartesianController::on_init() {
  auto node = get_node();

  side_name_ = node->get_parameter("side").as_string();
  control_mode_interface_ = node->get_parameter("control_mode").as_string();
  arm_name_ = side_name_ + "_arm";
  auto const &send_provider = side_name_ == "left" ? LEFT_SEND_PROVIDER : RIGHT_SEND_PROVIDER;
  int8_t const mode = control_mode_interface_ == CARTESIAN_IMPEDANCE_INTERFACE
                          ? victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE
                          : victor_lcm_interface::control_mode::CARTESIAN_POSE;

  send_lcm_ptr_ = std::make_shared<lcm::LCM>(send_provider);
  params_helper_ = std::make_shared<ControlModeParamsHelper>(node, LCMPtrs{send_lcm_ptr_}, mode);

  // ROS subscriber for receiving commands
  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Pose>(
      arm_name_ + "/cartesian_pose", 10,
      [this](geometry_msgs::msg::Pose::SharedPtr msg) { latest_cmd_msg_.emplace(*msg); });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration KukaCartesianController::command_interface_configuration() const {
  // Claim the cartesian command interfaces for the specified side
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(side_name_ + "/" + CARTESIAN_XT_INTERFACE);
  config.names.push_back(side_name_ + "/" + CARTESIAN_YT_INTERFACE);
  config.names.push_back(side_name_ + "/" + CARTESIAN_ZT_INTERFACE);
  config.names.push_back(side_name_ + "/" + CARTESIAN_WR_INTERFACE);
  config.names.push_back(side_name_ + "/" + CARTESIAN_XR_INTERFACE);
  config.names.push_back(side_name_ + "/" + CARTESIAN_YR_INTERFACE);
  config.names.push_back(side_name_ + "/" + CARTESIAN_ZR_INTERFACE);
  config.names.push_back(side_name_ + "/" + control_mode_interface_);
  return config;
}

controller_interface::InterfaceConfiguration KukaCartesianController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration(controller_interface::interface_configuration_type::NONE);
}

controller_interface::CallbackReturn KukaCartesianController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KukaCartesianController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto const &update_result = params_helper_->updateControlModeParams();
  if (!update_result.first) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), update_result.second);
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn KukaCartesianController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type KukaCartesianController::update(const rclcpp::Time &time,
                                                                  const rclcpp::Duration &period) {
  if (!latest_cmd_msg_) {
    RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "No command received yet...");
    return controller_interface::return_type::OK;
  }

  // Copy the values from the latest command message to the command interfaces.
  // This will cause the values bound to the command interfaces to be updated.
  // Specifically, the values will be updated in the hardware interface.
  command_interfaces_[0].set_value(latest_cmd_msg_->position.x);
  command_interfaces_[1].set_value(latest_cmd_msg_->position.y);
  command_interfaces_[2].set_value(latest_cmd_msg_->position.z);
  command_interfaces_[3].set_value(latest_cmd_msg_->orientation.w);
  command_interfaces_[4].set_value(latest_cmd_msg_->orientation.x);
  command_interfaces_[5].set_value(latest_cmd_msg_->orientation.y);
  command_interfaces_[6].set_value(latest_cmd_msg_->orientation.z);

  return controller_interface::return_type::OK;
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaCartesianController, controller_interface::ControllerInterface)
