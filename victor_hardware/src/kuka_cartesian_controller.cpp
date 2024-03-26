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

  // Parameters for the cartesian control modes
  node->declare_parameter<double>("kuka.path_execution.max_velocity.x", 75.0);
  node->declare_parameter<double>("kuka.path_execution.max_velocity.y", 75.0);
  node->declare_parameter<double>("kuka.path_execution.max_velocity.z", 75.0);
  node->declare_parameter<double>("kuka.path_execution.max_velocity.a", 15.0);
  node->declare_parameter<double>("kuka.path_execution.max_velocity.b", 15.0);
  node->declare_parameter<double>("kuka.path_execution.max_velocity.c", 15.0);
  node->declare_parameter<double>("kuka.path_execution.max_velocity.nullspace", 750.0);
  node->declare_parameter<double>("kuka.path_execution.max_acceleration.x", 0.1);
  node->declare_parameter<double>("kuka.path_execution.max_acceleration.y", 0.1);
  node->declare_parameter<double>("kuka.path_execution.max_acceleration.z", 0.1);
  node->declare_parameter<double>("kuka.path_execution.max_acceleration.a", 0.1);
  node->declare_parameter<double>("kuka.path_execution.max_acceleration.b", 0.1);
  node->declare_parameter<double>("kuka.path_execution.max_acceleration.c", 0.1);
  node->declare_parameter<double>("kuka.path_execution.max_acceleration.nullspace", 1.0);
  node->declare_parameter<double>("kuka.limits.max_path_deviation.x", 999.0);
  node->declare_parameter<double>("kuka.limits.max_path_deviation.y", 999.0);
  node->declare_parameter<double>("kuka.limits.max_path_deviation.z", 999.0);
  node->declare_parameter<double>("kuka.limits.max_path_deviation.a", 999.0);
  node->declare_parameter<double>("kuka.limits.max_path_deviation.b", 999.0);
  node->declare_parameter<double>("kuka.limits.max_path_deviation.c", 999.0);
  node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.x", 100.0);
  node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.y", 100.0);
  node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.z", 100.0);
  node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.a", 30.0);
  node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.b", 30.0);
  node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.c", 30.0);
  node->declare_parameter<double>("kuka.limits.max_control_force.x", 20.0);
  node->declare_parameter<double>("kuka.limits.max_control_force.y", 20.0);
  node->declare_parameter<double>("kuka.limits.max_control_force.z", 20.0);
  node->declare_parameter<double>("kuka.limits.max_control_force.a", 20.0);
  node->declare_parameter<double>("kuka.limits.max_control_force.b", 20.0);
  node->declare_parameter<double>("kuka.limits.max_control_force.c", 20.0);
  node->declare_parameter<bool>("kuka.limits.stop_on_max_control_force", false);
  if (control_mode_interface_ == CARTESIAN_IMPEDANCE_INTERFACE) {
    node->declare_parameter<double>("kuka.impedance.damping.x", 0.25);
    node->declare_parameter<double>("kuka.impedance.damping.y", 0.25);
    node->declare_parameter<double>("kuka.impedance.damping.z", 0.25);
    node->declare_parameter<double>("kuka.impedance.damping.a", 0.25);
    node->declare_parameter<double>("kuka.impedance.damping.b", 0.25);
    node->declare_parameter<double>("kuka.impedance.damping.c", 0.25);
    node->declare_parameter<double>("kuka.impedance.damping.nullspace", 0.75);
    node->declare_parameter<double>("kuka.impedance.stiffness.x", 50000.0);
    node->declare_parameter<double>("kuka.impedance.stiffness.y", 50000.0);
    node->declare_parameter<double>("kuka.impedance.stiffness.z", 50000.0);
    node->declare_parameter<double>("kuka.impedance.stiffness.a", 300.0);
    node->declare_parameter<double>("kuka.impedance.stiffness.b", 300.0);
    node->declare_parameter<double>("kuka.impedance.stiffness.c", 300.0);
    node->declare_parameter<double>("kuka.impedance.stiffness.nullspace", 100.0);
  }

  set_parameters_handle_ = node->add_on_set_parameters_callback([&](std::vector<rclcpp::Parameter> const &params) {
    rcl_interfaces::msg::SetParametersResult result{};
    result.successful = true;
    result.reason = "Success";

    // This callback gets call when any parameter is updated, but we only care about params starting with "kuka",
    // So first filter out the params we don't care about.
    std::vector<rclcpp::Parameter> kuka_ros_params;
    std::copy_if(params.cbegin(), params.cend(), std::back_inserter(kuka_ros_params),
                 [](const rclcpp::Parameter &parameter) { return parameter.get_name().find("kuka") == 0; });

    if (kuka_ros_params.empty()) {
      return result;
    }

    auto &params_msg = params_helper_->params();
    for (const auto &param : kuka_ros_params) {
      RCLCPP_INFO_STREAM(get_node()->get_logger(),
                         "Updating param: " << param.get_name() << " to " << param.value_to_string());

      if (param.get_name() == "kuka.path_execution.max_velocity.x") {
        params_msg.cartesian_path_execution_params.max_velocity.x = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.y") {
        params_msg.cartesian_path_execution_params.max_velocity.y = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.z") {
        params_msg.cartesian_path_execution_params.max_velocity.z = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.a") {
        params_msg.cartesian_path_execution_params.max_velocity.a = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.b") {
        params_msg.cartesian_path_execution_params.max_velocity.b = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.c") {
        params_msg.cartesian_path_execution_params.max_velocity.c = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.nullspace") {
        params_msg.cartesian_path_execution_params.max_nullspace_velocity = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.x") {
        params_msg.cartesian_path_execution_params.max_acceleration.x = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.y") {
        params_msg.cartesian_path_execution_params.max_acceleration.y = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.z") {
        params_msg.cartesian_path_execution_params.max_acceleration.z = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.a") {
        params_msg.cartesian_path_execution_params.max_acceleration.a = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.b") {
        params_msg.cartesian_path_execution_params.max_acceleration.b = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.c") {
        params_msg.cartesian_path_execution_params.max_acceleration.c = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.nullspace") {
        params_msg.cartesian_path_execution_params.max_nullspace_acceleration = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.x") {
        params_msg.cartesian_control_mode_limits.max_path_deviation.x = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.y") {
        params_msg.cartesian_control_mode_limits.max_path_deviation.y = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.z") {
        params_msg.cartesian_control_mode_limits.max_path_deviation.z = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.a") {
        params_msg.cartesian_control_mode_limits.max_path_deviation.a = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.b") {
        params_msg.cartesian_control_mode_limits.max_path_deviation.b = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.c") {
        params_msg.cartesian_control_mode_limits.max_path_deviation.c = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.x") {
        params_msg.cartesian_control_mode_limits.max_cartesian_velocity.x = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.y") {
        params_msg.cartesian_control_mode_limits.max_cartesian_velocity.y = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.z") {
        params_msg.cartesian_control_mode_limits.max_cartesian_velocity.z = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.a") {
        params_msg.cartesian_control_mode_limits.max_cartesian_velocity.a = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.b") {
        params_msg.cartesian_control_mode_limits.max_cartesian_velocity.b = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.c") {
        params_msg.cartesian_control_mode_limits.max_cartesian_velocity.c = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.x") {
        params_msg.cartesian_control_mode_limits.max_control_force.x = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.y") {
        params_msg.cartesian_control_mode_limits.max_control_force.y = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.z") {
        params_msg.cartesian_control_mode_limits.max_control_force.z = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.a") {
        params_msg.cartesian_control_mode_limits.max_control_force.a = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.b") {
        params_msg.cartesian_control_mode_limits.max_control_force.b = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.c") {
        params_msg.cartesian_control_mode_limits.max_control_force.c = param.as_double();
      }
      if (param.get_name() == "kuka.limits.stop_on_max_control_force") {
        params_msg.cartesian_control_mode_limits.stop_on_max_control_force = static_cast<int8_t>(param.as_bool());
      }
      if (param.get_name() == "kuka.impedance.damping.x") {
        params_msg.cartesian_impedance_params.cartesian_damping.x = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.damping.y") {
        params_msg.cartesian_impedance_params.cartesian_damping.y = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.damping.z") {
        params_msg.cartesian_impedance_params.cartesian_damping.z = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.damping.a") {
        params_msg.cartesian_impedance_params.cartesian_damping.a = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.damping.b") {
        params_msg.cartesian_impedance_params.cartesian_damping.b = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.damping.c") {
        params_msg.cartesian_impedance_params.cartesian_damping.c = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.damping.nullspace") {
        params_msg.cartesian_impedance_params.nullspace_damping = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.stiffness.x") {
        params_msg.cartesian_impedance_params.cartesian_stiffness.x = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.stiffness.y") {
        params_msg.cartesian_impedance_params.cartesian_stiffness.y = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.stiffness.z") {
        params_msg.cartesian_impedance_params.cartesian_stiffness.z = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.stiffness.a") {
        params_msg.cartesian_impedance_params.cartesian_stiffness.a = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.stiffness.b") {
        params_msg.cartesian_impedance_params.cartesian_stiffness.b = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.stiffness.c") {
        params_msg.cartesian_impedance_params.cartesian_stiffness.c = param.as_double();
      }
      if (param.get_name() == "kuka.impedance.stiffness.nullspace") {
        params_msg.cartesian_impedance_params.nullspace_stiffness = param.as_double();
      }
    }

    auto const &update_result = params_helper_->updateControlModeParams();
    if (!update_result.first) {
      result.successful = false;
      result.reason = update_result.second;
      return result;
    }

    return result;
  });

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
