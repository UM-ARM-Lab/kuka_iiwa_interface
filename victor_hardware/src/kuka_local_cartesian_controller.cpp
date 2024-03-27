#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <lifecycle_msgs/msg/state.hpp>
#include <string>
#include <vector>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/kuka_local_cartesian_controller.hpp>
#include <victor_hardware/validators.hpp>

namespace victor_hardware {

controller_interface::CallbackReturn KukaLocalCartesianController::on_init() {
  return KukaCartesianController::on_init();
}

controller_interface::InterfaceConfiguration KukaLocalCartesianController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interface;

  state_interface.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface.names.push_back(side_name_ + "/" + MEASURED_XT_STATE_INTERFACE);
  state_interface.names.push_back(side_name_ + "/" + MEASURED_YT_STATE_INTERFACE);
  state_interface.names.push_back(side_name_ + "/" + MEASURED_ZT_STATE_INTERFACE);
  state_interface.names.push_back(side_name_ + "/" + MEASURED_WR_STATE_INTERFACE);
  state_interface.names.push_back(side_name_ + "/" + MEASURED_XR_STATE_INTERFACE);
  state_interface.names.push_back(side_name_ + "/" + MEASURED_YR_STATE_INTERFACE);
  state_interface.names.push_back(side_name_ + "/" + MEASURED_ZR_STATE_INTERFACE);

  return state_interface;
}

controller_interface::return_type KukaLocalCartesianController::update(const rclcpp::Time &time,
                                                                       const rclcpp::Duration &period) {
  if (!latest_cmd_msg_) {
    RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "No command received yet...");
    return controller_interface::return_type::OK;
  }

  auto const &xt = state_interfaces_[0].get_value();
  auto const &yt = state_interfaces_[1].get_value();
  auto const &zt = state_interfaces_[2].get_value();
  auto const &wr = state_interfaces_[3].get_value();
  auto const &xr = state_interfaces_[4].get_value();
  auto const &yr = state_interfaces_[5].get_value();
  auto const &zr = state_interfaces_[6].get_value();

  // Convert to eigen vectors
  Eigen::Vector3d current_position(xt, yt, zt);
  Eigen::Quaterniond current_orientation(wr, xr, yr, zr);

  Eigen::Vector3d desired_position(latest_cmd_msg_->position.x, latest_cmd_msg_->position.y,
                                   latest_cmd_msg_->position.z);
  Eigen::Quaterniond desired_orientation(latest_cmd_msg_->orientation.w, latest_cmd_msg_->orientation.x,
                                         latest_cmd_msg_->orientation.y, latest_cmd_msg_->orientation.z);

  double const max_norm_trans = 0.05;
  double const norm_trans = (desired_position - current_position).norm();
  if (norm_trans > max_norm_trans) {
    desired_position = current_position + (desired_position - current_position) / norm_trans * max_norm_trans;
  }

  double const max_norm_rot_rad = 0.05;
  double const norm_max_rad = current_orientation.angularDistance(desired_orientation);
  if (norm_max_rad > max_norm_rot_rad) {
    desired_orientation = current_orientation.slerp(max_norm_rot_rad / norm_max_rad, desired_orientation);
  }

  command_interfaces_[0].set_value(desired_position.x());
  command_interfaces_[1].set_value(desired_position.y());
  command_interfaces_[2].set_value(desired_position.z());
  command_interfaces_[3].set_value(desired_orientation.w());
  command_interfaces_[4].set_value(desired_orientation.x());
  command_interfaces_[5].set_value(desired_orientation.y());
  command_interfaces_[6].set_value(desired_orientation.z());

  return controller_interface::return_type::OK;
}

}  // namespace victor_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(victor_hardware::KukaLocalCartesianController, controller_interface::ControllerInterface)
