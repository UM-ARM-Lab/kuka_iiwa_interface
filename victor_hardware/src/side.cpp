#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/lcm_ros_conversions.hpp>
#include <victor_hardware/side.hpp>
#include <victor_hardware/validators.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace victor_hardware {

Side::Side(std::string const& name)
    : side_name_(name), logger_(rclcpp::get_logger("VictorHardwareInterface." + name)) {}

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

  auto const ns = "/victor/" + side_name_ + "_arm/";
  motion_status_pub_ = node->create_publisher<msg::MotionStatus>(ns + DEFAULT_MOTION_STATUS_TOPIC, 10);
  gripper_status_pub_ = node->create_publisher<msg::Robotiq3FingerStatus>(ns + DEFAULT_GRIPPER_STATUS_TOPIC, 10);
  gripper_command_sub_ = node->create_subscription<msg::Robotiq3FingerCommand>(
      ns + DEFAULT_GRIPPER_COMMAND_TOPIC, 1, std::bind(&Side::gripperCommandROSCallback, this, _1), getter_options);

  control_mode_client_ = std::make_shared<KukaControlModeClientNode>(node, LEFT_RECV_PROVIDER, LEFT_SEND_PROVIDER);

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

hardware_interface::return_type Side::send_motion_command() {
  auto const& any_nan =
      std::any_of(hw_cmd_position_.begin(), hw_cmd_position_.end(), [](auto const& val) { return std::isnan(val); });

  if (any_nan) {
    // Just do nothing, not an error. Usually just means no controllers have started yet.
    return hardware_interface::return_type::OK;
  }

  auto const now = std::chrono::system_clock::now();
  auto const now_tp = std::chrono::time_point_cast<std::chrono::seconds>(now);
  std::chrono::duration<double> const now_dur_seconds = now_tp.time_since_epoch();
  auto const now_seconds = now_dur_seconds.count();

  victor_lcm_interface::motion_command motion_cmd{};
  motion_cmd.timestamp = now_seconds;
  // Set this based on which controller is currently running, changed only in the prepare_command_mode_switch function
  motion_cmd.control_mode.mode = static_cast<int8_t>(hw_cmd_control_mode_);
  motion_cmd.joint_position.joint_1 = hw_cmd_position_[0];
  motion_cmd.joint_position.joint_2 = hw_cmd_position_[1];
  motion_cmd.joint_position.joint_3 = hw_cmd_position_[2];
  motion_cmd.joint_position.joint_4 = hw_cmd_position_[3];
  motion_cmd.joint_position.joint_5 = hw_cmd_position_[4];
  motion_cmd.joint_position.joint_6 = hw_cmd_position_[5];
  motion_cmd.joint_position.joint_7 = hw_cmd_position_[6];
  motion_cmd.cartesian_pose.xt = hw_cmd_cartesian_pose_.position.x;
  motion_cmd.cartesian_pose.yt = hw_cmd_cartesian_pose_.position.y;
  motion_cmd.cartesian_pose.zt = hw_cmd_cartesian_pose_.position.z;
  motion_cmd.cartesian_pose.wr = hw_cmd_cartesian_pose_.orientation.w;
  motion_cmd.cartesian_pose.xr = hw_cmd_cartesian_pose_.orientation.x;
  motion_cmd.cartesian_pose.yr = hw_cmd_cartesian_pose_.orientation.y;
  motion_cmd.cartesian_pose.zr = hw_cmd_cartesian_pose_.orientation.z;
  motion_cmd.joint_velocity.joint_1 = 0.;
  motion_cmd.joint_velocity.joint_2 = 0.;
  motion_cmd.joint_velocity.joint_3 = 0.;
  motion_cmd.joint_velocity.joint_4 = 0.;
  motion_cmd.joint_velocity.joint_5 = 0.;
  motion_cmd.joint_velocity.joint_6 = 0.;
  motion_cmd.joint_velocity.joint_7 = 0.;

  auto const& active_control_mode = control_mode_client_->getControlMode();

  const auto validity_check_results = validateMotionCommand(active_control_mode.control_mode.mode, motion_cmd);
  if (validity_check_results.first) {
    send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &motion_cmd);
  } else {
    RCLCPP_ERROR_STREAM(logger_, "Arm motion command failed validity checks: " << validity_check_results.second);
  }

  return hardware_interface::return_type::OK;
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
void Side::add_state_interfaces(std::vector<hardware_interface::StateInterface>& state_interfaces) {
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.x", &hw_ft_[0]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.y", &hw_ft_[1]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.z", &hw_ft_[2]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.x", &hw_ft_[3]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.y", &hw_ft_[4]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.z", &hw_ft_[5]);
}

void Side::add_command_interfaces(std::vector<hardware_interface::CommandInterface>& command_interfaces) {
  // Control mode interface
  command_interfaces.emplace_back(side_name_, "control_mode", &hw_cmd_control_mode_);

  // Cartesian pose interfaces
  command_interfaces.emplace_back(side_name_, "cartesian_pose/xt", &hw_cmd_cartesian_pose_.position.x);
  command_interfaces.emplace_back(side_name_, "cartesian_pose/yt", &hw_cmd_cartesian_pose_.position.y);
  command_interfaces.emplace_back(side_name_, "cartesian_pose/zt", &hw_cmd_cartesian_pose_.position.z);
  command_interfaces.emplace_back(side_name_, "cartesian_pose/wr", &hw_cmd_cartesian_pose_.orientation.w);
  command_interfaces.emplace_back(side_name_, "cartesian_pose/xr", &hw_cmd_cartesian_pose_.orientation.x);
  command_interfaces.emplace_back(side_name_, "cartesian_pose/yr", &hw_cmd_cartesian_pose_.orientation.y);
  command_interfaces.emplace_back(side_name_, "cartesian_pose/zr", &hw_cmd_cartesian_pose_.orientation.z);

  // Joint interfaces
  for (uint i = 0; i < 7; i++) {
    // The interface names should be consistent with ros2_controllers.yaml and URDF
    command_interfaces.emplace_back("victor_" + side_name_ + "arm_joint_" + std::to_string(i),
                                    hardware_interface::HW_IF_POSITION, &hw_cmd_position_[i]);
  }
}

}  // namespace victor_hardware
