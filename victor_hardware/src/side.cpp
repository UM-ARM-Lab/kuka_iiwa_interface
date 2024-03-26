#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <victor_hardware/constants.hpp>
#include <victor_hardware/lcm_ros_conversions.hpp>
#include <victor_hardware/side.hpp>
#include <victor_hardware/validators.hpp>

using namespace std::placeholders;

namespace victor_hardware {

Side::Side(std::string const& name) : side_name_(name), logger_(rclcpp::get_logger("VictorHardwareInterface." + name)) {
  hw_state_ft_.fill(0.0);
}

void Side::add_state_interfaces(std::vector<hardware_interface::StateInterface>& state_interfaces) {
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.x", &hw_state_ft_[0]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.y", &hw_state_ft_[1]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "force.z", &hw_state_ft_[2]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.x", &hw_state_ft_[3]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.y", &hw_state_ft_[4]);
  state_interfaces.emplace_back(side_name_ + "_force_torque_sensor", "torque.z", &hw_state_ft_[5]);

  state_interfaces.emplace_back(side_name_, MEASURED_XT_STATE_INTERFACE, &hw_state_cartesian_pose_.position.x);
  state_interfaces.emplace_back(side_name_, MEASURED_YT_STATE_INTERFACE, &hw_state_cartesian_pose_.position.y);
  state_interfaces.emplace_back(side_name_, MEASURED_ZT_STATE_INTERFACE, &hw_state_cartesian_pose_.position.z);
  state_interfaces.emplace_back(side_name_, MEASURED_WR_STATE_INTERFACE, &hw_state_cartesian_pose_.orientation.w);
  state_interfaces.emplace_back(side_name_, MEASURED_XR_STATE_INTERFACE, &hw_state_cartesian_pose_.orientation.x);
  state_interfaces.emplace_back(side_name_, MEASURED_YR_STATE_INTERFACE, &hw_state_cartesian_pose_.orientation.y);
  state_interfaces.emplace_back(side_name_, MEASURED_ZR_STATE_INTERFACE, &hw_state_cartesian_pose_.orientation.z);

  state_interfaces.emplace_back(side_name_, COMMANDED_XT_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.position.x);
  state_interfaces.emplace_back(side_name_, COMMANDED_YT_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.position.y);
  state_interfaces.emplace_back(side_name_, COMMANDED_ZT_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.position.z);
  state_interfaces.emplace_back(side_name_, COMMANDED_WR_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.orientation.w);
  state_interfaces.emplace_back(side_name_, COMMANDED_XR_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.orientation.x);
  state_interfaces.emplace_back(side_name_, COMMANDED_YR_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.orientation.y);
  state_interfaces.emplace_back(side_name_, COMMANDED_ZR_STATE_INTERFACE, &hw_state_cmd_cartesian_pose_.orientation.z);
}

void Side::add_command_interfaces(hardware_interface::HardwareInfo const& info,
                                  std::vector<hardware_interface::CommandInterface>& command_interfaces) {
  // Cartesian pose interfaces
  command_interfaces.emplace_back(side_name_, CARTESIAN_XT_INTERFACE, &motion_cmd_.cartesian_pose.xt);
  command_interfaces.emplace_back(side_name_, CARTESIAN_YT_INTERFACE, &motion_cmd_.cartesian_pose.yt);
  command_interfaces.emplace_back(side_name_, CARTESIAN_ZT_INTERFACE, &motion_cmd_.cartesian_pose.zt);
  command_interfaces.emplace_back(side_name_, CARTESIAN_WR_INTERFACE, &motion_cmd_.cartesian_pose.wr);
  command_interfaces.emplace_back(side_name_, CARTESIAN_XR_INTERFACE, &motion_cmd_.cartesian_pose.xr);
  command_interfaces.emplace_back(side_name_, CARTESIAN_YR_INTERFACE, &motion_cmd_.cartesian_pose.yr);
  command_interfaces.emplace_back(side_name_, CARTESIAN_ZR_INTERFACE, &motion_cmd_.cartesian_pose.zr);

  // The interface names should be consistent with ros2_controllers.yaml and URDF
  auto const& prefix = "victor_" + side_name_ + "_arm_";
  command_interfaces.emplace_back(prefix + "joint_1", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.joint_position.joint_1);
  command_interfaces.emplace_back(prefix + "joint_2", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.joint_position.joint_2);
  command_interfaces.emplace_back(prefix + "joint_3", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.joint_position.joint_3);
  command_interfaces.emplace_back(prefix + "joint_4", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.joint_position.joint_4);
  command_interfaces.emplace_back(prefix + "joint_5", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.joint_position.joint_5);
  command_interfaces.emplace_back(prefix + "joint_6", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.joint_position.joint_6);
  command_interfaces.emplace_back(prefix + "joint_7", hardware_interface::HW_IF_POSITION,
                                  &motion_cmd_.joint_position.joint_7);

  // Command interfaces to represent the control modes
  command_interfaces.emplace_back(side_name_, JOINT_POSITION_INTERFACE, &hw_cmd_joint_position_control_mode_);
  command_interfaces.emplace_back(side_name_, JOINT_IMPEDANCE_INTERFACE, &hw_cmd_joint_impedance_control_mode_);
  command_interfaces.emplace_back(side_name_, CARTESIAN_POSE_INTERFACE, &hw_cmd_cartesian_pose_control_mode_);
  command_interfaces.emplace_back(side_name_, CARTESIAN_IMPEDANCE_INTERFACE, &hw_cmd_cartesian_impedance_control_mode_);
}

CallbackReturn Side::on_init(std::shared_ptr<rclcpp::Node> const& node, std::string const& send_provider,
                             std::string const& recv_provider) {
  hw_state_ft_.fill(0);
  motion_cmd_.joint_velocity.joint_1 = 0.;
  motion_cmd_.joint_velocity.joint_2 = 0.;
  motion_cmd_.joint_velocity.joint_3 = 0.;
  motion_cmd_.joint_velocity.joint_4 = 0.;
  motion_cmd_.joint_velocity.joint_5 = 0.;
  motion_cmd_.joint_velocity.joint_6 = 0.;
  motion_cmd_.joint_velocity.joint_7 = 0.;
  motion_cmd_.joint_position.joint_1 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.joint_position.joint_2 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.joint_position.joint_3 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.joint_position.joint_4 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.joint_position.joint_5 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.joint_position.joint_6 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.joint_position.joint_7 = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.cartesian_pose.xt = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.cartesian_pose.yt = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.cartesian_pose.zt = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.cartesian_pose.wr = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.cartesian_pose.xr = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.cartesian_pose.yr = std::numeric_limits<double>::quiet_NaN();
  motion_cmd_.cartesian_pose.zr = std::numeric_limits<double>::quiet_NaN();
  has_active_controller_ = false;
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
  control_mode_params_pub_ =
      node->create_publisher<msg::ControlModeParameters>(ns + DEFAULT_CONTROL_MODE_PARAMETERS_TOPIC, 10);
  gripper_status_pub_ = node->create_publisher<msg::Robotiq3FingerStatus>(ns + DEFAULT_GRIPPER_STATUS_TOPIC, 10);
  gripper_command_sub_ = node->create_subscription<msg::Robotiq3FingerCommand>(
      ns + DEFAULT_GRIPPER_COMMAND_TOPIC, 1, std::bind(&Side::gripperCommandROSCallback, this, _1), getter_options);

  control_mode_client_ = std::make_shared<KukaControlModeClientNode>(
      node, recv_lcm_ptr_, send_lcm_ptr_, [&](victor_lcm_interface::control_mode_parameters const& lcm_msg) {
        auto const& ros_msg = controlModeParamsLcmToRos(lcm_msg);
        control_mode_params_pub_->publish(ros_msg);
      });

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

void Side::read_motion_status(const victor_lcm_interface::motion_status& status) {
  // Copy the estimated external force torque readings from the motion status into the state interface
  hw_state_ft_[0] = status.estimated_external_wrench.x;
  hw_state_ft_[1] = status.estimated_external_wrench.y;
  hw_state_ft_[2] = status.estimated_external_wrench.z;
  hw_state_ft_[3] = status.estimated_external_wrench.a;
  hw_state_ft_[4] = status.estimated_external_wrench.b;
  hw_state_ft_[5] = status.estimated_external_wrench.c;

  hw_state_cartesian_pose_.position.x = status.measured_cartesian_pose.xt;
  hw_state_cartesian_pose_.position.y = status.measured_cartesian_pose.yt;
  hw_state_cartesian_pose_.position.z = status.measured_cartesian_pose.zt;
  hw_state_cartesian_pose_.orientation.w = status.measured_cartesian_pose.wr;
  hw_state_cartesian_pose_.orientation.x = status.measured_cartesian_pose.xr;
  hw_state_cartesian_pose_.orientation.y = status.measured_cartesian_pose.yr;
  hw_state_cartesian_pose_.orientation.z = status.measured_cartesian_pose.zr;

  hw_state_cmd_cartesian_pose_.position.x = status.commanded_cartesian_pose.xt;
  hw_state_cmd_cartesian_pose_.position.y = status.commanded_cartesian_pose.yt;
  hw_state_cmd_cartesian_pose_.position.z = status.commanded_cartesian_pose.zt;
  hw_state_cmd_cartesian_pose_.orientation.w = status.commanded_cartesian_pose.wr;
  hw_state_cmd_cartesian_pose_.orientation.x = status.commanded_cartesian_pose.xr;
  hw_state_cmd_cartesian_pose_.orientation.y = status.commanded_cartesian_pose.yr;
  hw_state_cmd_cartesian_pose_.orientation.z = status.commanded_cartesian_pose.zr;
}

hardware_interface::return_type Side::send_motion_command() {
  if (!has_active_controller_) {
    RCLCPP_INFO(logger_, "No active controller yet, not sending motion command");
    return hardware_interface::return_type::OK;
  }

  // Make sure the current active control mode matches what the latest mode_switch says it should be
  auto const& current_control_mode = control_mode_client_->getControlMode().control_mode.mode;
  if (latest_control_mode_ != current_control_mode) {
    RCLCPP_ERROR(logger_, "Current control mode does not match active control mode, not sending motion command");
    return hardware_interface::return_type::ERROR;
  }

  auto const now = std::chrono::system_clock::now();
  auto const now_tp = std::chrono::time_point_cast<std::chrono::seconds>(now);
  std::chrono::duration<double> const now_dur_seconds = now_tp.time_since_epoch();
  auto const now_seconds = now_dur_seconds.count();

  motion_cmd_.timestamp = now_seconds;
  motion_cmd_.control_mode.mode = latest_control_mode_;

  const auto validity_check_results = validateMotionCommand(motion_cmd_);
  if (validity_check_results.first) {
    send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &motion_cmd_);
  } else {
    rclcpp::Clock clock;
    RCLCPP_WARN_STREAM_THROTTLE(logger_, clock, 5000,
                                "Invalid motion command, " << validity_check_results.second << ", not sending");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Side::perform_command_mode_switch(const std::vector<std::string>& start_interfaces) {
  // Iterate over the start interfaces, and if any of them are of the form "side_name/control_mode_interface",
  // then switch that side to that control mode
  has_active_controller_ = false;

  bool success = true;
  for (auto const& interface : start_interfaces) {
    if (interface == side_name_ + "/" + JOINT_POSITION_INTERFACE) {
      latest_control_mode_ = victor_lcm_interface::control_mode::JOINT_POSITION;
      success = control_mode_client_->updateControlMode(latest_control_mode_);
    } else if (interface == side_name_ + "/" + JOINT_IMPEDANCE_INTERFACE) {
      latest_control_mode_ = victor_lcm_interface::control_mode::JOINT_IMPEDANCE;
      success = control_mode_client_->updateControlMode(latest_control_mode_);
    } else if (interface == side_name_ + "/" + CARTESIAN_POSE_INTERFACE) {
      latest_control_mode_ = victor_lcm_interface::control_mode::CARTESIAN_POSE;
      success = control_mode_client_->updateControlMode(latest_control_mode_);
    } else if (interface == side_name_ + "/" + CARTESIAN_IMPEDANCE_INTERFACE) {
      latest_control_mode_ = victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE;
      success = control_mode_client_->updateControlMode(latest_control_mode_);
    }
  }

  if (!success) {
    return hardware_interface::return_type::ERROR;
  }

  reset_motion_cmd_to_current_measured();

  has_active_controller_ = true;

  return hardware_interface::return_type::OK;
}

void Side::reset_motion_cmd_to_current_measured() {
  auto const& status = motion_status_listener_->getLatestMessage();

  motion_cmd_.joint_position.joint_1 = status.measured_joint_position.joint_1;
  motion_cmd_.joint_position.joint_2 = status.measured_joint_position.joint_2;
  motion_cmd_.joint_position.joint_3 = status.measured_joint_position.joint_3;
  motion_cmd_.joint_position.joint_4 = status.measured_joint_position.joint_4;
  motion_cmd_.joint_position.joint_5 = status.measured_joint_position.joint_5;
  motion_cmd_.joint_position.joint_6 = status.measured_joint_position.joint_6;
  motion_cmd_.joint_position.joint_7 = status.measured_joint_position.joint_7;

  motion_cmd_.cartesian_pose.xt = status.measured_cartesian_pose.xt;
  motion_cmd_.cartesian_pose.yt = status.measured_cartesian_pose.yt;
  motion_cmd_.cartesian_pose.zt = status.measured_cartesian_pose.zt;
  motion_cmd_.cartesian_pose.wr = status.measured_cartesian_pose.wr;
  motion_cmd_.cartesian_pose.xr = status.measured_cartesian_pose.xr;
  motion_cmd_.cartesian_pose.yr = status.measured_cartesian_pose.yr;
  motion_cmd_.cartesian_pose.zr = status.measured_cartesian_pose.zr;
}

}  // namespace victor_hardware
