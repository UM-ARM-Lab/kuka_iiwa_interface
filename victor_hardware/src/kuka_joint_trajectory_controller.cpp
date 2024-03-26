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

  control_mode_interface_ = node->get_parameter("control_mode_interface").as_string();
  int8_t const mode = control_mode_interface_ == JOINT_IMPEDANCE_INTERFACE ? victor_lcm_interface::control_mode::JOINT_IMPEDANCE : victor_lcm_interface::control_mode::JOINT_POSITION;

  left_send_lcm_ptr_ = std::make_shared<lcm::LCM>(LEFT_SEND_PROVIDER);
  right_send_lcm_ptr_ = std::make_shared<lcm::LCM>(RIGHT_SEND_PROVIDER);
  params_helper_ = std::make_shared<ControlModeParamsHelper>(node, LCMPtrs{left_send_lcm_ptr_, right_send_lcm_ptr_}, mode);

  rcl_interfaces::msg::ParameterDescriptor relative_joint_velocity_desc;
  relative_joint_velocity_desc.description = "Relative velocity of the joints. 0 is slowest, 1 is fastest.";
  relative_joint_velocity_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  rcl_interfaces::msg::FloatingPointRange relative_joint_velocity_range;
  relative_joint_velocity_range.from_value = 0.0;
  relative_joint_velocity_range.to_value = 1.0;
  relative_joint_velocity_desc.floating_point_range.push_back(relative_joint_velocity_range);
  // NOTE: the defaults here may not match the defaults used to initialize kuka_mode_params_ in the constructor,
  //  so these defaults may not be used. Or maybe they would be "read" by get params but then be wrong?
  node->declare_parameter<double>("kuka.joint_relative_velocity", 0.1, relative_joint_velocity_desc);

  if (control_mode_interface_ == JOINT_IMPEDANCE_INTERFACE) {
    rcl_interfaces::msg::ParameterDescriptor stiffness_desc;
    stiffness_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    node->declare_parameter<double>("kuka.stiffness.joint_1", 600.0, stiffness_desc);
    node->declare_parameter<double>("kuka.stiffness.joint_2", 600.0, stiffness_desc);
    node->declare_parameter<double>("kuka.stiffness.joint_3", 300.0, stiffness_desc);
    node->declare_parameter<double>("kuka.stiffness.joint_4", 300.0, stiffness_desc);
    node->declare_parameter<double>("kuka.stiffness.joint_5", 100.0, stiffness_desc);
    node->declare_parameter<double>("kuka.stiffness.joint_6", 100.0, stiffness_desc);
    node->declare_parameter<double>("kuka.stiffness.joint_7", 50.0, stiffness_desc);
    rcl_interfaces::msg::ParameterDescriptor damping_desc;
    damping_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    node->declare_parameter<double>("kuka.damping.joint_1", 0.7, damping_desc);
    node->declare_parameter<double>("kuka.damping.joint_2", 0.7, damping_desc);
    node->declare_parameter<double>("kuka.damping.joint_3", 0.7, damping_desc);
    node->declare_parameter<double>("kuka.damping.joint_4", 0.7, damping_desc);
    node->declare_parameter<double>("kuka.damping.joint_5", 0.7, damping_desc);
    node->declare_parameter<double>("kuka.damping.joint_6", 0.7, damping_desc);
    node->declare_parameter<double>("kuka.damping.joint_7", 0.7, damping_desc);
  }

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

    auto &kuka_mode_params_ = params_helper_->params();
    for (const auto &param : kuka_params) {
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Setting " << param.get_name() << " " << param.value_to_string());
      if (param.get_name() == "kuka.joint_relative_velocity") {
        kuka_mode_params_.joint_path_execution_params.joint_relative_velocity = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.joint_1") {
        kuka_mode_params_.joint_impedance_params.joint_stiffness.joint_1 = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.joint_2") {
        kuka_mode_params_.joint_impedance_params.joint_stiffness.joint_2 = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.joint_3") {
        kuka_mode_params_.joint_impedance_params.joint_stiffness.joint_3 = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.joint_4") {
        kuka_mode_params_.joint_impedance_params.joint_stiffness.joint_4 = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.joint_5") {
        kuka_mode_params_.joint_impedance_params.joint_stiffness.joint_5 = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.joint_6") {
        kuka_mode_params_.joint_impedance_params.joint_stiffness.joint_6 = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.joint_7") {
        kuka_mode_params_.joint_impedance_params.joint_stiffness.joint_7 = param.as_double();
      }
      if (param.get_name() == "kuka.damping.joint_1") {
        kuka_mode_params_.joint_impedance_params.joint_damping.joint_1 = param.as_double();
      }
      if (param.get_name() == "kuka.damping.joint_2") {
        kuka_mode_params_.joint_impedance_params.joint_damping.joint_2 = param.as_double();
      }
      if (param.get_name() == "kuka.damping.joint_3") {
        kuka_mode_params_.joint_impedance_params.joint_damping.joint_3 = param.as_double();
      }
      if (param.get_name() == "kuka.damping.joint_4") {
        kuka_mode_params_.joint_impedance_params.joint_damping.joint_4 = param.as_double();
      }
      if (param.get_name() == "kuka.damping.joint_5") {
        kuka_mode_params_.joint_impedance_params.joint_damping.joint_5 = param.as_double();
      }
      if (param.get_name() == "kuka.damping.joint_6") {
        kuka_mode_params_.joint_impedance_params.joint_damping.joint_6 = param.as_double();
      }
      if (param.get_name() == "kuka.damping.joint_7") {
        kuka_mode_params_.joint_impedance_params.joint_damping.joint_7 = param.as_double();
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
