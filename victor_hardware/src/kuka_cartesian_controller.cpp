#include <string>
#include <vector>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_cartesian_controller.hpp>
#include <victor_hardware/validators.hpp>

namespace victor_hardware {

controller_interface::CallbackReturn KukaCartesianController::on_init() {
  auto node = get_node();

  side_name_ = node->get_parameter("side").as_string();
  arm_name_ = side_name_ + "_arm";
  auto const &recv_provider = side_name_ == "left" ? LEFT_RECV_PROVIDER : RIGHT_RECV_PROVIDER;
  auto const &send_provider = side_name_ == "left" ? LEFT_SEND_PROVIDER : RIGHT_SEND_PROVIDER;

  control_mode_client_ =
      std::make_shared<KukaControlModeClientLifecycleNode>(node, recv_provider, send_provider);

  // Parameters for the cartesian impedance controller
  rcl_interfaces::msg::ParameterDescriptor max_x_velocity_desc;
  max_x_velocity_desc.description = "velocities of the end effector in X. Not sure what the units are.";
  node->declare_parameter<double>("kuka.max_velocity.x", 75.0, max_x_velocity_desc);

  rcl_interfaces::msg::ParameterDescriptor max_y_velocity_desc;
  max_y_velocity_desc.description = "velocities of the end effector in Y. Not sure what the units are.";
  node->declare_parameter<double>("kuka.max_velocity.y", 75.0, max_y_velocity_desc);

  rcl_interfaces::msg::ParameterDescriptor max_z_velocity_desc;
  max_z_velocity_desc.description = "velocities of the end effector in Z. Not sure what the units are.";
  node->declare_parameter<double>("kuka.max_velocity.z", 75.0, max_z_velocity_desc);

  // TODO add the rest of the parameters...

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
      RCLCPP_INFO_STREAM(get_node()->get_logger(),
                         "Updating param: " << param.get_name() << " to " << param.value_to_string());

      if (param.get_name() == "kuka.max_velocity.x") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.x = param.as_double();
      }
      if (param.get_name() == "kuka.max_velocity.y") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.y = param.as_double();
      }
      if (param.get_name() == "kuka.max_velocity.z") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.z = param.as_double();
      }
    }

    auto const &validate_mode_result = validateControlMode(kuka_mode_params_);
    if (!validate_mode_result.first) {
      result.successful = false;
      result.reason = validate_mode_result.second;
      return result;
    }

    auto const &update_success = control_mode_client_->updateControlModeParameters(kuka_mode_params_);
    if (!update_success) {
      result.successful = false;
      result.reason = "Failed to update control mode";
    }

    return result;
  });

  // ROS subscriber for receiving commands
  cmd_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      arm_name_ + "/cartesian_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { latest_cmd_msg_.emplace(*msg); });

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
  RCLCPP_INFO(get_node()->get_logger(), "Activating KukaCartesianController");
  auto const &success =
      control_mode_client_->updateControlMode(victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE);
  if (!success) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set control mode to CARTESIAN_IMPEDANCE");
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
  command_interfaces_[0].set_value(latest_cmd_msg_->pose.position.x);
  command_interfaces_[1].set_value(latest_cmd_msg_->pose.position.y);
  command_interfaces_[2].set_value(latest_cmd_msg_->pose.position.z);
  command_interfaces_[3].set_value(latest_cmd_msg_->pose.orientation.w);
  command_interfaces_[4].set_value(latest_cmd_msg_->pose.orientation.x);
  command_interfaces_[5].set_value(latest_cmd_msg_->pose.orientation.y);
  command_interfaces_[6].set_value(latest_cmd_msg_->pose.orientation.z);

  return controller_interface::return_type::OK;
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaCartesianController, controller_interface::ControllerInterface)
