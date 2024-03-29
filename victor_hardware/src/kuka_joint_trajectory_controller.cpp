#include <algorithm>
#include <memory>
#include <string>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_joint_trajectory_controller.hpp>
#include <victor_hardware/lcm_ostream_operators.hpp>

namespace victor_hardware {

controller_interface::InterfaceConfiguration KukaJointTrajectoryController::command_interface_configuration() const {
  auto command_interface_configuration =
      joint_trajectory_controller::JointTrajectoryController::command_interface_configuration();

  command_interface_configuration.names.emplace_back("left/" + control_mode_interface_);
  command_interface_configuration.names.emplace_back("right/" + control_mode_interface_);

  return command_interface_configuration;
}

controller_interface::CallbackReturn KukaJointTrajectoryController::on_init() {
  auto node = get_node();

  control_mode_interface_ = node->get_parameter("control_mode").as_string();
  int8_t const mode = control_mode_interface_ == JOINT_IMPEDANCE_INTERFACE ? victor_lcm_interface::control_mode::JOINT_IMPEDANCE : victor_lcm_interface::control_mode::JOINT_POSITION;

  left_send_lcm_ptr_ = std::make_shared<lcm::LCM>(LEFT_SEND_PROVIDER);
  right_send_lcm_ptr_ = std::make_shared<lcm::LCM>(RIGHT_SEND_PROVIDER);
  params_helper_ = std::make_shared<ControlModeParamsHelper>(node, LCMPtrs{left_send_lcm_ptr_, right_send_lcm_ptr_}, mode);

  // Call the parent class's on_init() method
  return joint_trajectory_controller::JointTrajectoryController::on_init();
}

controller_interface::CallbackReturn KukaJointTrajectoryController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto const &update_result = params_helper_->updateControlModeParams();
  if (!update_result.first) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), update_result.second);
    return controller_interface::CallbackReturn::ERROR;
  }
  return JointTrajectoryController::on_activate(previous_state);
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaJointTrajectoryController, controller_interface::ControllerInterface)
