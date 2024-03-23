#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/lcm_ros_conversions.hpp>
#include <victor_hardware/side.hpp>
#include <victor_hardware/validators.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace victor_hardware {

Side::Side(std::string const& name) : name_(name), logger_(rclcpp::get_logger("VictorHardwareInterface." + name)) {}

CallbackReturn Side::on_init(std::shared_ptr<rclcpp::Executor> const& executor,
                             std::shared_ptr<rclcpp::Node> const& node, std::string const& send_provider,
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

  control_mode_client_ = std::make_shared<KukaControlModeClientNode>(node, LEFT_RECV_PROVIDER, LEFT_SEND_PROVIDER);
  current_control_mode_ = control_mode_client_->getControlMode();

  return CallbackReturn::SUCCESS;
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

void Side::send_motion_command(victor_lcm_interface::motion_command const& command) {
  auto const& active_control_mode = control_mode_client_->getControlMode();

  if (!send_motion_command_) {
    return;
  }

  const auto validity_check_results = validateMotionCommand(active_control_mode.control_mode.mode, command);
  if (validity_check_results.first) {
    send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &command);
  } else {
    RCLCPP_ERROR_STREAM(logger_, "Arm motion command failed validity checks: " << validity_check_results.second);
  }
}

std::pair<bool, std::string> Side::validate_and_switch_to_mode_for_interfaces(
    const std::vector<std::string>& start_interfaces) {
  auto is_cartesian = [](std::string const& interface_name) {
    return interface_name.find("cartesian") != std::string::npos;
  };
  auto is_joint = [](std::string const& interface_name) { return interface_name.find("joint") != std::string::npos; };

  auto const is_all_cartesian = std::all_of(start_interfaces.begin(), start_interfaces.end(), is_cartesian);
  auto const is_all_joint = std::all_of(start_interfaces.begin(), start_interfaces.end(), is_joint);

  if (!is_all_cartesian && !is_all_joint) {
    return {false, "The requested mode switch mixes both cartesian and position/pose interfaces!"};
  }

  return {true, ""};
}

}  // namespace victor_hardware
