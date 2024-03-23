#include <algorithm>
#include <memory>
#include <string>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_joint_position_trajectory_controller.hpp>
#include <victor_hardware/validators.hpp>

namespace victor_hardware {

controller_interface::InterfaceConfiguration KukaJointPositionTrajectoryController::command_interface_configuration()
    const {
  auto command_interface_configuration =
      joint_trajectory_controller::JointTrajectoryController::command_interface_configuration();

  command_interface_configuration.names.emplace_back("left/control_mode");
  command_interface_configuration.names.emplace_back("right/control_mode");

  return command_interface_configuration;
}

controller_interface::CallbackReturn KukaJointPositionTrajectoryController::on_init() {
  // Add custom control mode parameters. Each parameter will have a callback that updates the kuka_mode_params_ struct,
  // then sends that info to the robot using the LCM client, via the KukaControlModeClient class.

  // Each controller should be used for exactly one control mode, so we set that here:
  kuka_mode_params_.control_mode.mode = victor_lcm_interface::control_mode::JOINT_POSITION;

  auto node = get_node();

  left_control_mode_client_ =
      std::make_shared<KukaControlModeClientLifecycleNode>(node, LEFT_RECV_PROVIDER, LEFT_SEND_PROVIDER);
  right_control_mode_client_ =
      std::make_shared<KukaControlModeClientLifecycleNode>(node, RIGHT_RECV_PROVIDER, RIGHT_SEND_PROVIDER);

  rcl_interfaces::msg::ParameterDescriptor relative_joint_velocity_desc;
  relative_joint_velocity_desc.description = "Relative velocity of the joints. 0 is slowest, 1 is fastest.";
  relative_joint_velocity_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  rcl_interfaces::msg::FloatingPointRange relative_joint_velocity_range;
  relative_joint_velocity_range.from_value = 0.0;
  relative_joint_velocity_range.to_value = 1.0;
  relative_joint_velocity_desc.floating_point_range.push_back(relative_joint_velocity_range);
  node->declare_parameter<double>("kuka.joint_relative_velocity", 0.1, relative_joint_velocity_desc);

  set_parameters_handle_ = node->add_on_set_parameters_callback([&](std::vector<rclcpp::Parameter> const &params) {
    rcl_interfaces::msg::SetParametersResult result{};
    result.successful = true;
    result.reason = "Success";

    // This callback gets call when any parameter is updated, but we only care about params starting with "kuka",
    // So first filter out the params we don't care about.
    std::vector<rclcpp::Parameter> kuka_params;
    std::copy_if(params.cbegin(), params.cend(), std::back_inserter(kuka_params),
                 [](const rclcpp::Parameter &parameter) { return parameter.get_name().find("kuka") == 0; });

    if (kuka_params.empty()) {
      return result;
    }

    // Ensure that we do not change the control MODE here, only update the params.
    auto const &current_left_mode = left_control_mode_client_->getControlMode().control_mode.mode;
    auto const &current_right_mode = right_control_mode_client_->getControlMode().control_mode.mode;
    if (current_left_mode != victor_lcm_interface::control_mode::JOINT_POSITION) {
      result.successful = false;
      result.reason = "Left arm is in " + std::to_string(current_left_mode) + " mode, not JOINT_POSITION";
      return result;
    }
    if (current_right_mode != victor_lcm_interface::control_mode::JOINT_POSITION) {
      result.successful = false;
      result.reason = "Right arm is in " + std::to_string(current_right_mode) + " mode, not JOINT_POSITION";
      return result;
    }

    for (const auto &param : kuka_params) {
      if (param.get_name() == "kuka.joint_relative_velocity") {
        kuka_mode_params_.joint_path_execution_params.joint_relative_velocity = param.as_double();
      }
    }

    auto const &validate_mode_result = validateControlMode(kuka_mode_params_);
    if (!validate_mode_result.first) {
      result.successful = false;
      result.reason = validate_mode_result.second;
      return result;
    }

    auto const &left_success = left_control_mode_client_->updateControlMode(kuka_mode_params_);
    auto const &right_success = right_control_mode_client_->updateControlMode(kuka_mode_params_);

    if (!left_success) {
      result.successful = false;
      result.reason = "Failed to update control mode for left arm";
    }
    if (!right_success) {
      result.successful = false;
      result.reason = "Failed to update control mode for right arm";
    }

    return result;
  });

  // Call the parent class's on_init() method
  return joint_trajectory_controller::JointTrajectoryController::on_init();
}

controller_interface::CallbackReturn KukaJointPositionTrajectoryController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto left_mode_params = left_control_mode_client_->getControlMode();
  left_mode_params.control_mode.mode = victor_lcm_interface::control_mode::JOINT_POSITION;
  auto const &left_success = left_control_mode_client_->updateControlMode(left_mode_params);

  if (!left_success) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to switch to JOINT_POSITION mode");
    return controller_interface::CallbackReturn::ERROR;
  }

  auto right_mode_params = right_control_mode_client_->getControlMode();
  right_mode_params.control_mode.mode = victor_lcm_interface::control_mode::JOINT_POSITION;
  auto const &right_success = right_control_mode_client_->updateControlMode(right_mode_params);

  if (!right_success) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to switch to JOINT_POSITION mode");
    return controller_interface::CallbackReturn::ERROR;
  }

  return JointTrajectoryController::on_activate(previous_state);
}
controller_interface::return_type KukaJointPositionTrajectoryController::update(const rclcpp::Time &time,
                                                                                const rclcpp::Duration &period) {
  // set the control mode to JOINT_POSITION in the command interface
  for (auto &command_interface : command_interfaces_) {
    if (command_interface.get_interface_name() == "left/control_mode" ||
        command_interface.get_interface_name() == "right/control_mode") {
      command_interface.set_value(victor_lcm_interface::control_mode::JOINT_POSITION);
    }
  }

  return JointTrajectoryController::update(time, period);
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaJointPositionTrajectoryController,
                       controller_interface::ControllerInterface)
