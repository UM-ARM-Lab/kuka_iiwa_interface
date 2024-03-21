#include <rclcpp/rclcpp.hpp>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/lcm_ros_conversions.hpp>
#include <victor_hardware/side.hpp>
#include <victor_hardware/validators.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace victor_hardware {

Side::Side(std::string const& name) : name_(name), logger_(rclcpp::get_logger("VictorHardwareInterface." + name)) {}

CallbackReturn Side::on_init(std::shared_ptr<rclcpp::Node> const& node, std::string const& send_provider,
                             std::string const& recv_provider) {
  hw_ft_.fill(0);
  send_lcm_ptr_ = std::make_shared<lcm::LCM>(send_provider);
  recv_lcm_ptr_ = std::make_shared<lcm::LCM>(recv_provider);

  if (!send_lcm_ptr_->good()) {
    RCLCPP_ERROR(logger_, "Left Send LCM interface is not good");
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(logger_, "Left send LCM interface is good");

  if (!recv_lcm_ptr_->good()) {
    RCLCPP_ERROR(logger_, "Left Receive LCM interface is not good");
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(logger_, "Left receive LCM interface is good");

  motion_status_listener_ = std::make_unique<LcmListener<victor_lcm_interface::motion_status>>(
      recv_lcm_ptr_, DEFAULT_MOTION_STATUS_CHANNEL,
      [&](victor_lcm_interface::motion_status const& msg) { publish_motion_status(msg); });
  control_mode_listener_ = std::make_unique<LcmListener<victor_lcm_interface::control_mode_parameters>>(
      recv_lcm_ptr_, DEFAULT_CONTROL_MODE_STATUS_CHANNEL,
      [&](victor_lcm_interface::control_mode_parameters const& msg) {});  // nothing to do here
  gripper_status_listener_ = std::make_unique<LcmListener<victor_lcm_interface::robotiq_3finger_status>>(
      recv_lcm_ptr_, DEFAULT_GRIPPER_STATUS_CHANNEL,
      [&](victor_lcm_interface::robotiq_3finger_status const& msg) { publish_gripper_status(msg); });

  // ROS API
  getter_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  setter_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions getter_options;
  getter_options.callback_group = getter_callback_group_;

  auto const ns = "/victor/" + name_ + "_arm/";
  motion_status_pub_ = node->create_publisher<msg::MotionStatus>(ns + DEFAULT_MOTION_STATUS_TOPIC, 10);
  gripper_status_pub_ = node->create_publisher<msg::Robotiq3FingerStatus>(ns + DEFAULT_GRIPPER_STATUS_TOPIC, 10);
  gripper_command_sub_ = node->create_subscription<msg::Robotiq3FingerCommand>(
      ns + DEFAULT_GRIPPER_COMMAND_TOPIC, 1, std::bind(&Side::gripperCommandROSCallback, this, _1), getter_options);
  get_control_mode_server_ = node->create_service<srv::GetControlMode>(
      ns + DEFAULT_GET_CONTROL_MODE_SERVICE, std::bind(&Side::getControlMode, this, _1, _2),
      rmw_qos_profile_services_default, getter_callback_group_);
  set_control_mode_server_ = node->create_service<srv::SetControlMode>(
      ns + DEFAULT_SET_CONTROL_MODE_SERVICE, std::bind(&Side::setControlMode, this, _1, _2),
      rmw_qos_profile_services_default, setter_callback_group_);

  return CallbackReturn::SUCCESS;
}

void Side::getControlMode(const std::shared_ptr<srv::GetControlMode::Request>& request,
                          std::shared_ptr<srv::GetControlMode::Response> response) const {
  // Get the latest control mode from the LCM listener
  auto const& control_mode_params = control_mode_listener_->getLatestMessage();
  response->active_control_mode = controlModeParamsLcmToRos(control_mode_params);
}

void Side::setControlMode(const std::shared_ptr<srv::SetControlMode::Request>& request,
                          std::shared_ptr<srv::SetControlMode::Response> response) const {
  // publish the control mode to the LCM, then wait for the control mode to be received
  auto const lcm_command = controlModeParamsRosToLcm(request->new_control_mode);
  send_lcm_ptr_->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &lcm_command);

  // Check to see if the control mode has changed, and if it hasn't after a certain amount of time,
  // set response->success = false, set the error message, and log an error message
  auto const start_time = std::chrono::steady_clock::now();
  while (true) {
    auto const& control_mode_params = control_mode_listener_->getLatestMessage();
    if (control_mode_params.control_mode.mode == request->new_control_mode.control_mode.mode) {
      response->success = true;
      return;
    }
    if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(1)) {
      response->success = false;
      response->message = "Control mode change timed out";
      RCLCPP_WARN(logger_, "Control mode change timed out");
      return;
    }
  }
}

void Side::gripperCommandROSCallback(const msg::Robotiq3FingerCommand& command) {
  auto const lcm_command = gripperCommandRosToLcm(command);
  send_lcm_ptr_->publish(DEFAULT_GRIPPER_COMMAND_CHANNEL, &lcm_command);
}

void Side::publish_motion_status(const victor_lcm_interface::motion_status& lcm_msg) {
  auto ros_msg = motionStatusLcmToRos(lcm_msg);
  motion_status_pub_->publish(ros_msg);
}

void Side::publish_gripper_status(const victor_lcm_interface::robotiq_3finger_status& lcm_msg) {
  auto ros_msg = gripperStatusLcmToRos(lcm_msg);
  gripper_status_pub_->publish(ros_msg);
}

}  // namespace victor_hardware
