#pragma once

#include <controller_interface/controller_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_control_mode_client.hpp>
#include <victor_hardware/lcm_ostream_operators.hpp>
#include <victor_hardware/types.hpp>
#include <victor_hardware/validators.hpp>
#include <victor_lcm_interface/control_mode_parameters.hpp>

using namespace std::placeholders;

namespace victor_hardware {

class ControlModeParamsHelper {
 public:
  using SharedPtr = std::shared_ptr<ControlModeParamsHelper>;

  explicit ControlModeParamsHelper(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::vector<LCMPtr> send_lcm_ptrs,
                                   int8_t const mode)
      : node_(node), send_lcm_ptrs_(send_lcm_ptrs) {
    kuka_mode_params_.control_mode.mode = mode;

    // Declare ROS parameters depending on the control mode
    if (mode == victor_lcm_interface::control_mode::JOINT_POSITION ||
        mode == victor_lcm_interface::control_mode::JOINT_IMPEDANCE) {
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

      if (mode == victor_lcm_interface::control_mode::JOINT_IMPEDANCE) {
        rcl_interfaces::msg::ParameterDescriptor stiffness_desc;
        stiffness_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        node->declare_parameter<double>("kuka.stiffness.joint_1", DEFAULT_JOINT1_STIFFNESS, stiffness_desc);
        node->declare_parameter<double>("kuka.stiffness.joint_2", DEFAULT_JOINT2_STIFFNESS, stiffness_desc);
        node->declare_parameter<double>("kuka.stiffness.joint_3", DEFAULT_JOINT3_STIFFNESS, stiffness_desc);
        node->declare_parameter<double>("kuka.stiffness.joint_4", DEFAULT_JOINT4_STIFFNESS, stiffness_desc);
        node->declare_parameter<double>("kuka.stiffness.joint_5", DEFAULT_JOINT5_STIFFNESS, stiffness_desc);
        node->declare_parameter<double>("kuka.stiffness.joint_6", DEFAULT_JOINT6_STIFFNESS, stiffness_desc);
        node->declare_parameter<double>("kuka.stiffness.joint_7", DEFAULT_JOINT7_STIFFNESS, stiffness_desc);
        rcl_interfaces::msg::ParameterDescriptor damping_desc;
        damping_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        node->declare_parameter<double>("kuka.damping.joint_1", DEFAULT_JOINT_DAMPING, damping_desc);
        node->declare_parameter<double>("kuka.damping.joint_2", DEFAULT_JOINT_DAMPING, damping_desc);
        node->declare_parameter<double>("kuka.damping.joint_3", DEFAULT_JOINT_DAMPING, damping_desc);
        node->declare_parameter<double>("kuka.damping.joint_4", DEFAULT_JOINT_DAMPING, damping_desc);
        node->declare_parameter<double>("kuka.damping.joint_5", DEFAULT_JOINT_DAMPING, damping_desc);
        node->declare_parameter<double>("kuka.damping.joint_6", DEFAULT_JOINT_DAMPING, damping_desc);
        node->declare_parameter<double>("kuka.damping.joint_7", DEFAULT_JOINT_DAMPING, damping_desc);
      }
    } else if (mode == victor_lcm_interface::control_mode::CARTESIAN_POSE ||
               mode == victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE) {
      // Parameters for the cartesian control modes
      node->declare_parameter<double>("kuka.path_execution.max_velocity.x", DEFAULT_MAX_LIN_VELOCITY);
      node->declare_parameter<double>("kuka.path_execution.max_velocity.y", DEFAULT_MAX_LIN_VELOCITY);
      node->declare_parameter<double>("kuka.path_execution.max_velocity.z", DEFAULT_MAX_LIN_VELOCITY);
      node->declare_parameter<double>("kuka.path_execution.max_velocity.a", DEFAULT_MAX_ROT_VELOCITY);
      node->declare_parameter<double>("kuka.path_execution.max_velocity.b", DEFAULT_MAX_ROT_VELOCITY);
      node->declare_parameter<double>("kuka.path_execution.max_velocity.c", DEFAULT_MAX_ROT_VELOCITY);
      node->declare_parameter<double>("kuka.path_execution.max_velocity.nullspace", DEFAULT_MAX_NULLSPACE_VELOCITY);
      node->declare_parameter<double>("kuka.path_execution.max_acceleration.x", DEFAULT_MAX_LIN_ACCELERATION);
      node->declare_parameter<double>("kuka.path_execution.max_acceleration.y", DEFAULT_MAX_LIN_ACCELERATION);
      node->declare_parameter<double>("kuka.path_execution.max_acceleration.z", DEFAULT_MAX_LIN_ACCELERATION);
      node->declare_parameter<double>("kuka.path_execution.max_acceleration.a", DEFAULT_MAX_ROT_ACCELERATION);
      node->declare_parameter<double>("kuka.path_execution.max_acceleration.b", DEFAULT_MAX_ROT_ACCELERATION);
      node->declare_parameter<double>("kuka.path_execution.max_acceleration.c", DEFAULT_MAX_ROT_ACCELERATION);
      node->declare_parameter<double>("kuka.path_execution.max_acceleration.nullspace",
                                      DEFAULT_MAX_NULLSPACE_ACCELERATION);
      node->declare_parameter<double>("kuka.limits.max_path_deviation.x", DEFAULT_MAX_PATH_DEVIATION);
      node->declare_parameter<double>("kuka.limits.max_path_deviation.y", DEFAULT_MAX_PATH_DEVIATION);
      node->declare_parameter<double>("kuka.limits.max_path_deviation.z", DEFAULT_MAX_PATH_DEVIATION);
      node->declare_parameter<double>("kuka.limits.max_path_deviation.a", DEFAULT_MAX_PATH_DEVIATION);
      node->declare_parameter<double>("kuka.limits.max_path_deviation.b", DEFAULT_MAX_PATH_DEVIATION);
      node->declare_parameter<double>("kuka.limits.max_path_deviation.c", DEFAULT_MAX_PATH_DEVIATION);
      node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.x", DEFAULT_MAX_LIN_VELOCITY);
      node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.y", DEFAULT_MAX_LIN_VELOCITY);
      node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.z", DEFAULT_MAX_LIN_VELOCITY);
      node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.a", DEFAULT_MAX_ROT_VELOCITY);
      node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.b", DEFAULT_MAX_ROT_VELOCITY);
      node->declare_parameter<double>("kuka.limits.max_cartesian_velocity.c", DEFAULT_MAX_ROT_VELOCITY);
      node->declare_parameter<double>("kuka.limits.max_control_force.x", DEFAULT_MAX_CONTROL_FORCE);
      node->declare_parameter<double>("kuka.limits.max_control_force.y", DEFAULT_MAX_CONTROL_FORCE);
      node->declare_parameter<double>("kuka.limits.max_control_force.z", DEFAULT_MAX_CONTROL_FORCE);
      node->declare_parameter<double>("kuka.limits.max_control_force.a", DEFAULT_MAX_CONTROL_FORCE);
      node->declare_parameter<double>("kuka.limits.max_control_force.b", DEFAULT_MAX_CONTROL_FORCE);
      node->declare_parameter<double>("kuka.limits.max_control_force.c", DEFAULT_MAX_CONTROL_FORCE);
      node->declare_parameter<bool>("kuka.limits.stop_on_max_control_force", DEFAULT_STOP_ON_MAX_CONTROL_FORCE);

      if (mode == victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE) {
        node->declare_parameter<double>("kuka.damping.x", DEFAULT_CARTESIAN_DAMPING);
        node->declare_parameter<double>("kuka.damping.y", DEFAULT_CARTESIAN_DAMPING);
        node->declare_parameter<double>("kuka.damping.z", DEFAULT_CARTESIAN_DAMPING);
        node->declare_parameter<double>("kuka.damping.a", DEFAULT_CARTESIAN_DAMPING);
        node->declare_parameter<double>("kuka.damping.b", DEFAULT_CARTESIAN_DAMPING);
        node->declare_parameter<double>("kuka.damping.c", DEFAULT_CARTESIAN_DAMPING);
        node->declare_parameter<double>("kuka.damping.nullspace", DEFAULT_CARTESIAN_DAMPING);
        node->declare_parameter<double>("kuka.stiffness.x", DEFAULT_CARTESIAN_STIFFNESS);
        node->declare_parameter<double>("kuka.stiffness.y", DEFAULT_CARTESIAN_STIFFNESS);
        node->declare_parameter<double>("kuka.stiffness.z", DEFAULT_CARTESIAN_STIFFNESS);
        node->declare_parameter<double>("kuka.stiffness.a", DEFAULT_CARTESIAN_STIFFNESS);
        node->declare_parameter<double>("kuka.stiffness.b", DEFAULT_CARTESIAN_STIFFNESS);
        node->declare_parameter<double>("kuka.stiffness.c", DEFAULT_CARTESIAN_STIFFNESS);
        node->declare_parameter<double>("kuka.stiffness.nullspace", DEFAULT_CARTESIAN_STIFFNESS);
      }
    }

    set_parameters_handle_ =
        node->add_on_set_parameters_callback(std::bind(&ControlModeParamsHelper::OnSetParametersCallback, this, _1));
  }

  rcl_interfaces::msg::SetParametersResult OnSetParametersCallback(std::vector<rclcpp::Parameter> const &params) {
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

    for (const auto &param : kuka_ros_params) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Updating param: " << param.get_name() << " to " << param.value_to_string());
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
      if (param.get_name() == "kuka.path_execution.max_velocity.x") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.x = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.y") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.y = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.z") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.z = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.a") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.a = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.b") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.b = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.c") {
        kuka_mode_params_.cartesian_path_execution_params.max_velocity.c = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_velocity.nullspace") {
        kuka_mode_params_.cartesian_path_execution_params.max_nullspace_velocity = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.x") {
        kuka_mode_params_.cartesian_path_execution_params.max_acceleration.x = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.y") {
        kuka_mode_params_.cartesian_path_execution_params.max_acceleration.y = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.z") {
        kuka_mode_params_.cartesian_path_execution_params.max_acceleration.z = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.a") {
        kuka_mode_params_.cartesian_path_execution_params.max_acceleration.a = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.b") {
        kuka_mode_params_.cartesian_path_execution_params.max_acceleration.b = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.c") {
        kuka_mode_params_.cartesian_path_execution_params.max_acceleration.c = param.as_double();
      }
      if (param.get_name() == "kuka.path_execution.max_acceleration.nullspace") {
        kuka_mode_params_.cartesian_path_execution_params.max_nullspace_acceleration = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.x") {
        kuka_mode_params_.cartesian_control_mode_limits.max_path_deviation.x = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.y") {
        kuka_mode_params_.cartesian_control_mode_limits.max_path_deviation.y = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.z") {
        kuka_mode_params_.cartesian_control_mode_limits.max_path_deviation.z = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.a") {
        kuka_mode_params_.cartesian_control_mode_limits.max_path_deviation.a = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.b") {
        kuka_mode_params_.cartesian_control_mode_limits.max_path_deviation.b = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_path_deviation.c") {
        kuka_mode_params_.cartesian_control_mode_limits.max_path_deviation.c = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.x") {
        kuka_mode_params_.cartesian_control_mode_limits.max_cartesian_velocity.x = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.y") {
        kuka_mode_params_.cartesian_control_mode_limits.max_cartesian_velocity.y = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.z") {
        kuka_mode_params_.cartesian_control_mode_limits.max_cartesian_velocity.z = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.a") {
        kuka_mode_params_.cartesian_control_mode_limits.max_cartesian_velocity.a = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.b") {
        kuka_mode_params_.cartesian_control_mode_limits.max_cartesian_velocity.b = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_cartesian_velocity.c") {
        kuka_mode_params_.cartesian_control_mode_limits.max_cartesian_velocity.c = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.x") {
        kuka_mode_params_.cartesian_control_mode_limits.max_control_force.x = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.y") {
        kuka_mode_params_.cartesian_control_mode_limits.max_control_force.y = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.z") {
        kuka_mode_params_.cartesian_control_mode_limits.max_control_force.z = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.a") {
        kuka_mode_params_.cartesian_control_mode_limits.max_control_force.a = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.b") {
        kuka_mode_params_.cartesian_control_mode_limits.max_control_force.b = param.as_double();
      }
      if (param.get_name() == "kuka.limits.max_control_force.c") {
        kuka_mode_params_.cartesian_control_mode_limits.max_control_force.c = param.as_double();
      }
      if (param.get_name() == "kuka.limits.stop_on_max_control_force") {
        kuka_mode_params_.cartesian_control_mode_limits.stop_on_max_control_force =
            static_cast<int8_t>(param.as_bool());
      }
      if (param.get_name() == "kuka.damping.x") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_damping.x = param.as_double();
      }
      if (param.get_name() == "kuka.damping.y") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_damping.y = param.as_double();
      }
      if (param.get_name() == "kuka.damping.z") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_damping.z = param.as_double();
      }
      if (param.get_name() == "kuka.damping.a") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_damping.a = param.as_double();
      }
      if (param.get_name() == "kuka.damping.b") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_damping.b = param.as_double();
      }
      if (param.get_name() == "kuka.damping.c") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_damping.c = param.as_double();
      }
      if (param.get_name() == "kuka.damping.nullspace") {
        kuka_mode_params_.cartesian_impedance_params.nullspace_damping = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.x") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.x = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.y") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.y = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.z") {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Setting stiffness z" << param.as_double());
        kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.z = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.a") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.a = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.b") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.b = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.c") {
        kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.c = param.as_double();
      }
      if (param.get_name() == "kuka.stiffness.nullspace") {
        kuka_mode_params_.cartesian_impedance_params.nullspace_stiffness = param.as_double();
      }
    }

    auto const &is_active = node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    if (is_active) {
      auto const &update_result = updateControlModeParams();
      if (!update_result.first) {
        result.successful = false;
        result.reason = update_result.second;
        return result;
      }
    }

    return result;
  }

  [[nodiscard]] ErrorType updateControlModeParams() {
    RCLCPP_DEBUG(node_->get_logger(), "Updating control mode params: ");
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "" << kuka_mode_params_);

    auto const &validate_mode_result = validateControlMode(kuka_mode_params_);
    if (!validate_mode_result.first) {
      return validate_mode_result;
    }

    for (auto const &send_lcm_ptr : send_lcm_ptrs_) {
      send_lcm_ptr->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &kuka_mode_params_);
    }

    return {true, ""};
  }

 private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  victor_lcm_interface::control_mode_parameters kuka_mode_params_ = default_control_mode_parameters();
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_;
  LCMPtrs send_lcm_ptrs_;
};

}  // namespace victor_hardware