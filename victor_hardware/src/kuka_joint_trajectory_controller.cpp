#include <memory>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>
#include <victor_hardware/kuka_joint_trajectory_controller.hpp>

namespace victor_hardware {

controller_interface::CallbackReturn KukaJointPositionTrajectoryController::on_init() {
  // Add custom control mode parameters
  auto node = get_node();

  rcl_interfaces::msg::ParameterDescriptor relative_joint_velocity_desc;
  relative_joint_velocity_desc.description = "Relative velocity of the joints. 0 is slowest, 1 is fastest.";
  rcl_interfaces::msg::FloatingPointRange relative_joint_velocity_range;
  relative_joint_velocity_range.from_value = 0.0;
  relative_joint_velocity_range.to_value = 1.0;
  relative_joint_velocity_desc.floating_point_range.push_back(relative_joint_velocity_range);

  node->declare_parameter<double>("kuka.joint_relative_velocity", DEFAULT_RELATIVE_VELOCITY,
                                  relative_joint_velocity_desc);

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node);

  handles_.emplace_back(
      param_subscriber_->add_parameter_callback("kuka.joint_relative_velocity", [&](const rclcpp::Parameter& p) {
        kuka_mode_params_.joint_path_execution_params.joint_relative_velocity = p.as_double();
        sendParamsLCM();
      }));

  // Call the parent class's on_init() method
  return joint_trajectory_controller::JointTrajectoryController::on_init();
}

void KukaJointPositionTrajectoryController::sendParamsLCM() {}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaJointPositionTrajectoryController,
                       controller_interface::ControllerInterface)
