#include <algorithm>
#include <memory>
#include <string>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_joint_position_trajectory_controller.hpp>
#include <victor_hardware/validators.hpp>

namespace victor_hardware {

// namespace victor_hardware

controller_interface::CallbackReturn KukaJointPositionTrajectoryController::on_init() {
  // Add custom control mode parameters. Each parameter will have a callback that updates the kuka_mode_params_ struct,
  // then sends that info to the robot using the LCM client, via the KukaControlModeClient class.

  // Each controller should be used for exactly one control mode, so we set that here:
  kuka_mode_params_.control_mode.mode = victor_lcm_interface::control_mode::JOINT_POSITION;

  auto node = get_node();

  left_control_mode_client_ = std::make_shared<KukaControlModeClient>(node, LEFT_RECV_PROVIDER, LEFT_SEND_PROVIDER);
  right_control_mode_client_ = std::make_shared<KukaControlModeClient>(node, RIGHT_RECV_PROVIDER, RIGHT_SEND_PROVIDER);

  rcl_interfaces::msg::ParameterDescriptor relative_joint_velocity_desc;
  relative_joint_velocity_desc.description = "Relative velocity of the joints. 0 is slowest, 1 is fastest.";
  rcl_interfaces::msg::FloatingPointRange relative_joint_velocity_range;
  relative_joint_velocity_range.from_value = 0.0;
  relative_joint_velocity_range.to_value = 1.0;
  relative_joint_velocity_desc.floating_point_range.push_back(relative_joint_velocity_range);

  node->declare_parameter<double>("kuka.joint_relative_velocity", DEFAULT_RELATIVE_VELOCITY,
                                  relative_joint_velocity_desc);

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

    RCLCPP_INFO(get_node()->get_logger(), "Updating KukaJointPositionTrajectoryController parameters");

    // Ensure that we do not change the control MODE here, only update the params.
    auto const &current_left_mode = left_control_mode_client_->getControlMode();
    auto const &current_right_mode = right_control_mode_client_->getControlMode();
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

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "modes are correct");

    for (const auto &param : kuka_params) {
      if (param.get_name() == "kuka.joint_relative_velocity") {
        kuka_mode_params_.joint_path_execution_params.joint_relative_velocity = param.as_double();
      }
    }

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Updating control modes");
    updateControlModes();
    return result;
  });

  // Call the parent class's on_init() method
  return joint_trajectory_controller::JointTrajectoryController::on_init();
}

void KukaJointPositionTrajectoryController::updateControlModes() {
  auto const &left_success = left_control_mode_client_->updateControlMode(kuka_mode_params_);
  auto const &right_success = right_control_mode_client_->updateControlMode(kuka_mode_params_);

  if (!left_success) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to update control mode for left arm");
  }
  if (!right_success) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to update control mode for right arm");
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Updated control modes successfully");
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaJointPositionTrajectoryController,
                       controller_interface::ControllerInterface)
