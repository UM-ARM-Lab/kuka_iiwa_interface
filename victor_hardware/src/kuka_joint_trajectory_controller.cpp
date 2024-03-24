#include <algorithm>
#include <memory>
#include <string>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_joint_trajectory_controller.hpp>
#include <victor_hardware/validators.hpp>

namespace victor_hardware {

controller_interface::InterfaceConfiguration KukaJointTrajectoryController::command_interface_configuration() const {
  auto command_interface_configuration =
      joint_trajectory_controller::JointTrajectoryController::command_interface_configuration();

  return command_interface_configuration;
}

controller_interface::CallbackReturn KukaJointTrajectoryController::on_init() {
  auto node = get_node();

  // Read this once at the beginning, use it in on_activate()
  control_mode_ = static_cast<int8_t>(node->get_parameter("mode").as_int());

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

    auto const &left_success = left_control_mode_client_->updateControlModeParameters(kuka_mode_params_);
    auto const &right_success = right_control_mode_client_->updateControlModeParameters(kuka_mode_params_);

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

controller_interface::CallbackReturn KukaJointTrajectoryController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto const &left_success = left_control_mode_client_->updateControlMode(control_mode_);
  if (!left_success) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to set left control mode to " << control_mode_);
    return controller_interface::CallbackReturn::ERROR;
  }

  auto const &right_success = right_control_mode_client_->updateControlMode(control_mode_);
  if (!right_success) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to set right control mode to " << control_mode_);
    return controller_interface::CallbackReturn::ERROR;
  }

  return JointTrajectoryController::on_activate(previous_state);
}
controller_interface::return_type KukaJointTrajectoryController::update(const rclcpp::Time &time,
                                                                        const rclcpp::Duration &period) {
  return JointTrajectoryController::update(time, period);
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaJointTrajectoryController, controller_interface::ControllerInterface)
