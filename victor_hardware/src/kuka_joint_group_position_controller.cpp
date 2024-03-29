#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_joint_group_position_controller.hpp>

namespace victor_hardware {

controller_interface::CallbackReturn KukaJointGroupPositionController::on_init() {
  auto node = get_node();

  control_mode_interface_ = node->get_parameter("control_mode").as_string();
  side_name_ = node->get_parameter("side").as_string();
  auto const& mode = control_mode_interface_ == JOINT_POSITION_INTERFACE
                         ? victor_lcm_interface::control_mode::JOINT_POSITION
                         : victor_lcm_interface::control_mode::JOINT_IMPEDANCE;
  auto const& send_provider = side_name_ == "left" ? LEFT_SEND_PROVIDER : RIGHT_SEND_PROVIDER;

  send_lcm_ptr_ = std::make_shared<lcm::LCM>(send_provider);
  params_helper_ = std::make_shared<ControlModeParamsHelper>(node, LCMPtrs{send_lcm_ptr_}, mode);

  return JointGroupPositionController::on_init();
}

controller_interface::InterfaceConfiguration KukaJointGroupPositionController::command_interface_configuration() const {
  auto command_interfaces = JointGroupPositionController::command_interface_configuration();
  command_interfaces.names.push_back(side_name_ + "/" + control_mode_interface_);
  return command_interfaces;
}

controller_interface::CallbackReturn KukaJointGroupPositionController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  auto const& update_result = params_helper_->updateControlModeParams();
  if (!update_result.first) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), update_result.second);
    return controller_interface::CallbackReturn::ERROR;
  }

  base_command_interface_configuration_ = JointGroupPositionController::command_interface_configuration();

  return JointGroupPositionController::on_activate(previous_state);
}

controller_interface::return_type KukaJointGroupPositionController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {

  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  auto const &expected_num_joints = base_command_interface_configuration_.names.size();
  if ((*joint_commands)->data.size() != expected_num_joints)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < expected_num_joints; ++index)
  {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaJointGroupPositionController, controller_interface::ControllerInterface)
