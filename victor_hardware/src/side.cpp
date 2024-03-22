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
  switch_controller_client_ = node->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controllers", rmw_qos_profile_services_default, setter_callback_group_);

  // Load and parse the ros2_controllers.yaml which is currently in victor_moveit_config
  auto const& package_share_directory = ament_index_cpp::get_package_share_directory("victor_moveit_config");
  auto const& controllers_file = package_share_directory + "/config/ros2_controllers.yaml";

  YAML::Node config = YAML::LoadFile(controllers_file);

  if (config["controller_manager"]) {
    if (!config["controller_manager"]["ros__parameters"]) {
      RCLCPP_ERROR(logger_, "controller_manager found in ros2_controllers.yaml, but no ros__parameters found");
      return CallbackReturn::ERROR;
    }
    auto const& controller_manager_config = config["controller_manager"]["ros__parameters"];
    for (auto const& controller : controller_manager_config) {
      auto const& controller_name = controller.first.as<std::string>();
      if (controller.second.Type() == YAML::NodeType::Map) {
        if (controller.second["control_modes"]) {
          auto const& control_modes = controller.second["control_modes"];
          std::vector<uint8_t> valid_control_modes;
          for (auto const& control_mode : control_modes) {
            valid_control_modes.push_back(control_mode.as<uint8_t>());
          }
          valid_modes_for_controllers_map_[controller_name] = valid_control_modes;
        }
      }
    }
  } else {
    RCLCPP_ERROR(logger_, "controller_manager not found in ros2_controllers.yaml");
    return CallbackReturn::ERROR;
  }

  current_control_mode_ = control_mode_listener_->getLatestMessage().control_mode.mode;

  return CallbackReturn::SUCCESS;
}

void Side::send_motion_command(victor_lcm_interface::motion_command const& command) {
  auto const& active_control_mode = control_mode_listener_->getLatestMessage().control_mode;

  if (!send_motion_command_) {
    return;
  }

  const auto validity_check_results = validateMotionCommand(active_control_mode.mode, command);
  if (validity_check_results.first) {
    send_lcm_ptr_->publish(DEFAULT_MOTION_COMMAND_CHANNEL, &command);
  } else {
    RCLCPP_ERROR_STREAM(logger_, "Arm motion command failed validity checks: " << validity_check_results.second);
  }
}

void Side::getControlMode(const std::shared_ptr<srv::GetControlMode::Request>& request,
                          std::shared_ptr<srv::GetControlMode::Response> response) const {
  // Get the latest control mode from the LCM listener
  auto const& control_mode_params = control_mode_listener_->getLatestMessage();
  response->active_control_mode = controlModeParamsLcmToRos(control_mode_params);
}

void Side::setControlMode(const std::shared_ptr<srv::SetControlMode::Request>& request,
                          std::shared_ptr<srv::SetControlMode::Response> response) {
  // Validate the request to make sure that the new controller and the new control mode are compatible

  auto const& [is_valid, error_msg] = validateControlModeRequest(*request);
  if (!is_valid) {
    RCLCPP_ERROR_STREAM(logger_, "Control mode request failed validity checks: " << error_msg);
    response->success = false;
    response->message = error_msg;
    return;
  }

  send_motion_command_ = false;  // prevent sending LCM motion commands

  // Set the KUKA control mode
  // publish the control mode to the LCM, then wait for the control mode to be received
  auto const lcm_command = controlModeParamsRosToLcm(request->new_control_mode);
  send_lcm_ptr_->publish(DEFAULT_CONTROL_MODE_COMMAND_CHANNEL, &lcm_command);

  // Check to see if the control mode has changed, and if it hasn't after a certain amount of time,
  // set response->success = false, set the error message, and log an error message
  auto const start_time = std::chrono::steady_clock::now();
  while (true) {
    auto const& control_mode_params = control_mode_listener_->getLatestMessage();
    auto const& current_control_mode = control_mode_params.control_mode.mode;
    if (current_control_mode == request->new_control_mode.control_mode.mode) {
      response->success = true;
      current_control_mode_ = current_control_mode;
      break;
    }
    if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(1)) {
      response->success = false;
      response->message = "Control mode change timed out";
      RCLCPP_WARN(logger_, "Control mode change timed out");
      return;
    }
  }

  // Switch ROS2 controllers
  while (!switch_controller_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Switch controller service not available");
      response->success = false;
      response->message = "Switch controller service not available";
      return;
    }
    RCLCPP_WARN(logger_, "Switch controller service not available, waiting again...");
  }

  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers.push_back(request->new_controller_name);

  // figure out what to put in the deactivate_controllers field by:
  // Getting a reference to the resource manager, the list of active controllers, and their command interfaces
  // iterating over each currently active controller and comparing their command interfaces to the new controller's
  // command interfaces, and if they don't match, add them to the deactivate_controllers field
  // req->deactivate_controllers

  auto result = switch_controller_client_->async_send_request(req);
  if (result.wait_for(1s) != std::future_status::ready) {
    auto const srv_error_msg = "No response from switch controller service";
    RCLCPP_ERROR(logger_, srv_error_msg);
    response->success = false;
    response->message = srv_error_msg;
    return;
  }

  auto const& switch_controller_response = result.get();
  if (!switch_controller_response->ok) {
    auto const srv_error_msg = "Switch controller service call did not succeed";
    RCLCPP_ERROR(logger_, srv_error_msg);
    response->success = false;
    response->message = srv_error_msg;
    return;
  }

  send_motion_command_ = true;  // resume sending LCM motion commands
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

std::pair<bool, std::string> Side::validateControlModeRequest(
    const victor_hardware_interfaces::srv::SetControlMode::Request& request) {
  auto const& mode = request.new_control_mode.control_mode.mode;

  if (request.new_controller_name.empty()) {
    return {false, "New controller name is empty"};
  }

  auto const& it = valid_modes_for_controllers_map_.find(request.new_controller_name);
  if (it == valid_modes_for_controllers_map_.end()) {
    auto const error_msg = "Controller " + request.new_controller_name + " not found.";
    return {false, error_msg};
  }

  auto const& valid_control_modes_for_new_controller = it->second;
  if (std::find(valid_control_modes_for_new_controller.begin(), valid_control_modes_for_new_controller.end(), mode) ==
      valid_control_modes_for_new_controller.end()) {
    return {false, "Control mode not valid for controller"};
  }

  return {true, ""};
}
std::pair<bool, std::string> Side::validate_mode_switch(const std::vector<std::string>& start_interfaces) {
  auto is_cartesian = [](std::string const& interface_name) {
    return interface_name.find("cartesian") != std::string::npos;
  };
  auto is_joint = [](std::string const& interface_name) { return interface_name.find("joint") != std::string::npos; };

  auto const is_all_cartesian = std::all_of(start_interfaces.begin(), start_interfaces.end(), is_cartesian);
  auto const is_all_joint = std::all_of(start_interfaces.begin(), start_interfaces.end(), is_joint);
  auto const current_mode_is_cartesian =
      current_control_mode_ == victor_lcm_interface::control_mode::CARTESIAN_POSE ||
      current_control_mode_ == victor_lcm_interface::control_mode::CARTESIAN_IMPEDANCE;
  auto const current_mode_is_joint = current_control_mode_ == victor_lcm_interface::control_mode::JOINT_POSITION ||
                                     current_control_mode_ == victor_lcm_interface::control_mode::JOINT_IMPEDANCE;

  auto const& current_mode_str = std::to_string(current_control_mode_);
  if (is_all_cartesian && current_mode_is_cartesian) {
    return {true, ""};
  } else if (is_all_cartesian) {
    return {false, "Cannot switch to cartesian controller with current mode " + current_mode_str};
  }

  if (is_all_joint && current_mode_is_joint) {
    return {true, ""};
  } else if (is_all_joint) {
    return {false, "Cannot switch to joint controller with current mode " + current_mode_str};
  }

  return {false, "Cannot switch to controller which uses both cartesian and joint interfaces"};
}

}  // namespace victor_hardware
