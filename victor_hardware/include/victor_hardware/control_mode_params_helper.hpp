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
    } else if (mode == victor_lcm_interface::control_mode::CARTESIAN_POSE ||
               mode == victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE) {
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

      if (mode == victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE) {
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

      for (const auto &param : kuka_ros_params) {
        RCLCPP_DEBUG_STREAM(node->get_logger(),
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
        if (param.get_name() == "kuka.impedance.damping.x") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_damping.x = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.damping.y") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_damping.y = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.damping.z") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_damping.z = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.damping.a") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_damping.a = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.damping.b") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_damping.b = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.damping.c") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_damping.c = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.damping.nullspace") {
          kuka_mode_params_.cartesian_impedance_params.nullspace_damping = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.stiffness.x") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.x = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.stiffness.y") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.y = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.stiffness.z") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.z = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.stiffness.a") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.a = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.stiffness.b") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.b = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.stiffness.c") {
          kuka_mode_params_.cartesian_impedance_params.cartesian_stiffness.c = param.as_double();
        }
        if (param.get_name() == "kuka.impedance.stiffness.nullspace") {
          kuka_mode_params_.cartesian_impedance_params.nullspace_stiffness = param.as_double();
        }
      }

      auto const &update_result = updateControlModeParams();
      if (!update_result.first) {
        result.successful = false;
        result.reason = update_result.second;
        return result;
      }

      return result;
    });
  }

  ErrorType updateControlModeParams() {
    // only do this if the controller is active!
    if (node_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return {true, ""};
    }

    auto const &validate_mode_result = validateControlMode(kuka_mode_params_);
    if (!validate_mode_result.first) {
      return validate_mode_result;
    }

    RCLCPP_INFO(node_->get_logger(), "Updating control mode params: ");
    RCLCPP_INFO_STREAM(node_->get_logger(), kuka_mode_params_);

    // Send it a bunch of times to make sure it gets there, and to stall to let the change take place...
    // Since the HW IF will be reading and sending the new params
    for (auto i{0}; i < 10; ++i) {
      for (auto const &send_lcm_ptr : send_lcm_ptrs_) {
        send_lcm_ptr->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &kuka_mode_params_);
      }

      usleep(1000);
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