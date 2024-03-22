#include <memory>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>
#include <victor_hardware/kuka_joint_trajectory_controller.hpp>

namespace victor_hardware {

controller_interface::CallbackReturn KukaJointPositionTrajectoryController::on_init() {
  // Use this to add custom control mode parameters
  auto node = get_node();
  node->declare_parameter<double>("stiffness", 200);

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node);

  auto cb = [&](const rclcpp::Parameter& p) {
    RCLCPP_INFO(node->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%f\"",
                p.get_name().c_str(), p.get_type_name().c_str(), p.as_double());
  };
  cb_handle_ = param_subscriber_->add_parameter_callback("stiffness", cb);

  return joint_trajectory_controller::JointTrajectoryController::on_init();
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaJointPositionTrajectoryController, controller_interface::ControllerInterface)
