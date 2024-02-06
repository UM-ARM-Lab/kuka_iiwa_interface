#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

// ROS message headers
#include <victor_hardware_interfaces/msg/control_mode_parameters.hpp>
#include <victor_hardware_interfaces/msg/motion_command.hpp>
#include <victor_hardware_interfaces/msg/motion_status.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_command.hpp>
#include <victor_hardware_interfaces/msg/robotiq3_finger_status.hpp>
#include <victor_hardware_interfaces/srv/get_control_mode.hpp>
#include <victor_hardware_interfaces/srv/set_control_mode.hpp>

// LCM
#include <lcm/lcm-cpp.hpp>

// Classes to speak to each individual hardware element
#include <victor_hardware/constants.hpp>
#include <victor_hardware/iiwa_lcm_bridge.hpp>
#include <victor_hardware/robotiq_3f_lcm_bridge.hpp>
#include <victor_hardware/victor_lcm_bridge_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace victor_hardware;
namespace msg = victor_hardware_interfaces::msg;
namespace srv = victor_hardware_interfaces::srv;

auto const logger = rclcpp::get_logger("victor_lcm_bridge_node");


class IiwaLcmBridgeNode : public rclcpp::Node {
 public:
  IiwaLcmBridgeNode() : Node("victor_hardware_node") {
    // Declare parameters
    // this->declare_parameter("cartesian_control_frame", DEFAULT_CARTESIAN_CONTROL_FRAME);
    this->declare_parameter(CARTESIAN_CONTROL_FRAME_PARAM, DEFAULT_CARTESIAN_CONTROL_FRAME);
    this->declare_parameter(SET_CONTROL_MODE_TIMEOUT_PARAM, DEFAULT_SET_CONTROL_MODE_TIMEOUT);
    this->declare_parameter(MOTION_COMMAND_TOPIC_PARAM, DEFAULT_MOTION_COMMAND_TOPIC);
    this->declare_parameter(MOTION_STATUS_TOPIC_PARAM, DEFAULT_MOTION_STATUS_TOPIC);
    this->declare_parameter(CONTROL_MODE_STATUS_TOPIC_PARAM, DEFAULT_CONTROL_MODE_STATUS_TOPIC);
    this->declare_parameter(GET_CONTROL_MODE_SERVICE_PARAM, DEFAULT_GET_CONTROL_MODE_SERVICE);
    this->declare_parameter(SET_CONTROL_MODE_SERVICE_PARAM, DEFAULT_SET_CONTROL_MODE_SERVICE);
    this->declare_parameter(GRIPPER_COMMAND_TOPIC_PARAM, DEFAULT_GRIPPER_COMMAND_TOPIC);
    this->declare_parameter(GRIPPER_STATUS_TOPIC_PARAM, DEFAULT_GRIPPER_STATUS_TOPIC);
    this->declare_parameter(SEND_LCM_URL_PARAM, DEFAULT_SEND_LCM_URL);
    this->declare_parameter(RECV_LCM_URL_PARAM, DEFAULT_RECV_LCM_URL);
    this->declare_parameter(MOTION_COMMAND_CHANNEL_PARAM, DEFAULT_MOTION_COMMAND_CHANNEL);
    this->declare_parameter(MOTION_STATUS_CHANNEL_PARAM, DEFAULT_MOTION_STATUS_CHANNEL);
    this->declare_parameter(CONTROL_MODE_COMMAND_CHANNEL_PARAM, DEFAULT_CONTROL_MODE_COMMAND_CHANNEL);
    this->declare_parameter(CONTROL_MODE_STATUS_CHANNEL_PARAM, DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
    this->declare_parameter(GRIPPER_COMMAND_CHANNEL_PARAM, DEFAULT_GRIPPER_COMMAND_CHANNEL);
    this->declare_parameter(GRIPPER_STATUS_CHANNEL_PARAM, DEFAULT_GRIPPER_STATUS_CHANNEL);

    // Get paramter values
    cartesian_control_frame_ = this->get_parameter(CARTESIAN_CONTROL_FRAME_PARAM).as_string();
    set_control_mode_timeout_ = this->get_parameter(SET_CONTROL_MODE_TIMEOUT_PARAM).as_double();
    auto const motion_command_topic = this->get_parameter(MOTION_COMMAND_TOPIC_PARAM).as_string();
    auto const motion_status_topic = this->get_parameter(MOTION_STATUS_TOPIC_PARAM).as_string();
    auto const control_mode_status_topic = this->get_parameter(CONTROL_MODE_STATUS_TOPIC_PARAM).as_string();
    auto const get_control_mode_service = this->get_parameter(GET_CONTROL_MODE_SERVICE_PARAM).as_string();
    auto const set_control_mode_service = this->get_parameter(SET_CONTROL_MODE_SERVICE_PARAM).as_string();
    auto const gripper_command_topic = this->get_parameter(GRIPPER_COMMAND_TOPIC_PARAM).as_string();
    auto const gripper_status_topic = this->get_parameter(GRIPPER_STATUS_TOPIC_PARAM).as_string();
    auto const send_lcm_url = this->get_parameter(SEND_LCM_URL_PARAM).as_string();
    auto const recv_lcm_url = this->get_parameter(RECV_LCM_URL_PARAM).as_string();
    auto const motion_command_channel = this->get_parameter(MOTION_COMMAND_CHANNEL_PARAM).as_string();
    auto const motion_status_channel = this->get_parameter(MOTION_STATUS_CHANNEL_PARAM).as_string();
    auto const control_mode_command_channel = this->get_parameter(CONTROL_MODE_COMMAND_CHANNEL_PARAM).as_string();
    auto const control_mode_status_channel = this->get_parameter(CONTROL_MODE_STATUS_CHANNEL_PARAM).as_string();
    auto const gripper_command_channel = this->get_parameter(GRIPPER_COMMAND_CHANNEL_PARAM).as_string();
    auto const gripper_status_channel = this->get_parameter(GRIPPER_STATUS_CHANNEL_PARAM).as_string();


    RCLCPP_INFO(logger, "Starting with send [%s] and receive [%s] LCM...", send_lcm_url.c_str(), recv_lcm_url.c_str());
    send_lcm_ptr_ = std::make_shared<lcm::LCM>(send_lcm_url);
    recv_lcm_ptr_ = std::make_shared<lcm::LCM>(recv_lcm_url);

    // Set up IIWA LCM bridge
    const auto motion_status_callback_fn = [&](const msg::MotionStatus& motion_status) {
      return motionStatusLCMCallback(motion_status);
    };
    const auto control_mode_status_callback_fn = [&](const msg::ControlModeParameters& control_mode_status) {
      return controlModeStatusLCMCallback(control_mode_status);
    };
    iiwa_ptr_ = std::make_unique<IiwaLcmBridge>(
        send_lcm_ptr_, recv_lcm_ptr_, motion_command_channel, motion_status_channel, motion_status_callback_fn,
        control_mode_command_channel, control_mode_status_channel, control_mode_status_callback_fn);

    // Set up Robotiq LCM interface
    const auto gripper_status_callback_fn = [&](const msg::Robotiq3FingerStatus& gripper_status) {
      return gripperStatusLCMCallback(gripper_status);
    };
    robotiq_ptr_ = std::make_unique<Robotiq3fLcmBridge>(send_lcm_ptr_, recv_lcm_ptr_, gripper_command_channel,
                                                        gripper_status_channel, gripper_status_callback_fn);

    // Set up ROS interfaces
    motion_status_pub_ = create_publisher<msg::MotionStatus>(motion_status_topic, 1);
    control_mode_status_pub_ = create_publisher<msg::ControlModeParameters>(control_mode_status_topic, 1);
    gripper_status_pub_ = create_publisher<msg::Robotiq3FingerStatus>(gripper_status_topic, 1);
    motion_command_sub_ = create_subscription<msg::MotionCommand>(
        motion_command_topic, 1, std::bind(&IiwaLcmBridgeNode::motionCommandROSCallback, this, _1));
    gripper_command_sub_ = create_subscription<msg::Robotiq3FingerCommand>(
        gripper_command_topic, 1, std::bind(&IiwaLcmBridgeNode::gripperCommandROSCallback, this, _1));
    set_control_mode_server_ = create_service<srv::SetControlMode>(
        set_control_mode_service, std::bind(&IiwaLcmBridgeNode::setControlModeCallback, this, _1, _2));
    get_control_mode_server_ = create_service<srv::GetControlMode>(
        get_control_mode_service, std::bind(&IiwaLcmBridgeNode::getControlModeCallback, this, _1, _2));

    // Create a timer callback that runs every 1 millisecond
    auto timer_callback = [&]() { handle_lcm(); };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(LCM_HANDLE_TIMEOUT), timer_callback);
  }

  void handle_lcm() {
    const int ret = recv_lcm_ptr_->handleTimeout(LCM_HANDLE_TIMEOUT);
    if (ret < 0) {
      RCLCPP_ERROR_STREAM(logger, "LCM error: " << ret);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Get/Set Control Mode functionality
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //// Static helpers to check parameters for being in valid ranges for the control mode /////////////////////////

  static std::pair<bool, std::string> validateJointPathExecutionParams(
      const msg::JointPathExecutionParameters& params) {
    bool valid = true;
    std::string message;
    if (params.joint_relative_velocity <= 0.0 || params.joint_relative_velocity > 1.0) {
      valid = false;
      message += "+Invalid joint relative velocity";
    }
    if (params.joint_relative_acceleration <= 0.0 || params.joint_relative_acceleration > 1.0) {
      valid = false;
      message += "+Invalid joint relative acceleration";
    }
    if (params.override_joint_acceleration < 0.0 || params.override_joint_acceleration > 10.0) {
      valid = false;
      message += "+Invalid override joint acceleration";
    }
    return {valid, message};
  }

  std::pair<bool, std::string> validateCartesianPathExecutionParams(
      const msg::CartesianPathExecutionParameters& params) {
    bool valid = true;
    std::string message;

    // Velocities, mm/s, rad/s, and 1/s respectively
    if (params.max_velocity.x <= 0.0) {
      valid = false;
      message += "+Invalid DoF X max velocity";
    }
    if (params.max_velocity.y <= 0.0) {
      valid = false;
      message += "+Invalid DoF Y max velocity";
    }
    if (params.max_velocity.z <= 0.0) {
      valid = false;
      message += "+Invalid DoF Z max velocity";
    }
    if (params.max_velocity.a <= 0.0) {
      valid = false;
      message += "+Invalid DoF A max velocity";
    }
    if (params.max_velocity.b <= 0.0) {
      valid = false;
      message += "+Invalid DoF B max velocity";
    }
    if (params.max_velocity.c <= 0.0) {
      valid = false;
      message += "+Invalid DoF C max velocity";
    }
    if (params.max_nullspace_velocity <= 0.0) {
      valid = false;
      message += "+Invalid nullspace max velocity";
    }

    // Accelerations, mm/s^2, rad/s^2, and 1/s^2 respectively
    if (params.max_acceleration.x <= 0.0) {
      valid = false;
      message += "+Invalid DoF X max acceleration";
    }
    if (params.max_acceleration.y <= 0.0) {
      valid = false;
      message += "+Invalid DoF Y max acceleration";
    }
    if (params.max_acceleration.z <= 0.0) {
      valid = false;
      message += "+Invalid DoF Z max acceleration";
    }
    if (params.max_acceleration.a <= 0.0) {
      valid = false;
      message += "+Invalid DoF A max acceleration";
    }
    if (params.max_acceleration.b <= 0.0) {
      valid = false;
      message += "+Invalid DoF B max acceleration";
    }
    if (params.max_acceleration.c <= 0.0) {
      valid = false;
      message += "+Invalid DoF C max acceleration";
    }
    if (params.max_nullspace_acceleration <= 0.0) {
      valid = false;
      message += "+Invalid nullspace max acceleration";
    }

    return {valid, message};
  }

  std::pair<bool, std::string> validateJointImpedanceParams(const msg::JointImpedanceParameters& params) {
    bool valid = true;
    std::string message;

    // Joint damping - unitless
    if (params.joint_damping.joint_1 < 0.0 || params.joint_damping.joint_1 > 1.0) {
      valid = false;
      message += "+Invalid joint 1 damping";
    }
    if (params.joint_damping.joint_2 < 0.0 || params.joint_damping.joint_2 > 1.0) {
      valid = false;
      message += "+Invalid joint 2 damping";
    }
    if (params.joint_damping.joint_3 < 0.0 || params.joint_damping.joint_3 > 1.0) {
      valid = false;
      message += "+Invalid joint 3 damping";
    }
    if (params.joint_damping.joint_4 < 0.0 || params.joint_damping.joint_4 > 1.0) {
      valid = false;
      message += "+Invalid joint 4 damping";
    }
    if (params.joint_damping.joint_5 < 0.0 || params.joint_damping.joint_5 > 1.0) {
      valid = false;
      message += "+Invalid joint 5 damping";
    }
    if (params.joint_damping.joint_6 < 0.0 || params.joint_damping.joint_6 > 1.0) {
      valid = false;
      message += "+Invalid joint 6 damping";
    }
    if (params.joint_damping.joint_7 < 0.0 || params.joint_damping.joint_7 > 1.0) {
      valid = false;
      message += "+Invalid joint 7 damping";
    }

    // Joint stiffness - Nm/rad
    if (params.joint_stiffness.joint_1 < 0.0) {
      valid = false;
      message += "+Invalid joint 1 stiffness";
    }
    if (params.joint_stiffness.joint_2 < 0.0) {
      valid = false;
      message += "+Invalid joint 2 stiffness";
    }
    if (params.joint_stiffness.joint_3 < 0.0) {
      valid = false;
      message += "+Invalid joint 3 stiffness";
    }
    if (params.joint_stiffness.joint_4 < 0.0) {
      valid = false;
      message += "+Invalid joint 4 stiffness";
    }
    if (params.joint_stiffness.joint_5 < 0.0) {
      valid = false;
      message += "+Invalid joint 5 stiffness";
    }
    if (params.joint_stiffness.joint_6 < 0.0) {
      valid = false;
      message += "+Invalid joint 6 stiffness";
    }
    if (params.joint_stiffness.joint_7 < 0.0) {
      valid = false;
      message += "+Invalid joint 7 stiffness";
    }

    return {valid, message};
  }

  std::pair<bool, std::string> validateCartesianImpedanceParams(const msg::CartesianImpedanceParameters& params) {
    bool valid = true;
    std::string message;

    // Damping - unitless
    if (params.cartesian_damping.x < 0.1 || params.cartesian_damping.x > 1.0) {
      valid = false;
      message += "+Invalid DoF X damping";
    }
    if (params.cartesian_damping.y < 0.1 || params.cartesian_damping.y > 1.0) {
      valid = false;
      message += "+Invalid DoF Y damping";
    }
    if (params.cartesian_damping.z < 0.1 || params.cartesian_damping.z > 1.0) {
      valid = false;
      message += "+Invalid DoF Z damping";
    }
    if (params.cartesian_damping.a < 0.1 || params.cartesian_damping.a > 1.0) {
      valid = false;
      message += "+Invalid DoF A damping";
    }
    if (params.cartesian_damping.b < 0.1 || params.cartesian_damping.b > 1.0) {
      valid = false;
      message += "+Invalid DoF B damping";
    }
    if (params.cartesian_damping.c < 0.1 || params.cartesian_damping.c > 1.0) {
      valid = false;
      message += "+Invalid DoF C damping";
    }
    if (params.nullspace_damping < 0.3 || params.nullspace_damping > 1.0) {
      valid = false;
      message += "+Invalid nullspace damping";
    }

    // Stiffness - units N/m, Nm/rad, no idea for nullspace
    if (params.cartesian_stiffness.x < 0.0 || params.cartesian_stiffness.x > 5000.0) {
      valid = false;
      message += "+Invalid DoF X stiffness";
    }
    if (params.cartesian_stiffness.y < 0.0 || params.cartesian_stiffness.y > 5000.0) {
      valid = false;
      message += "+Invalid DoF Y stiffness";
    }
    if (params.cartesian_stiffness.z < 0.0 || params.cartesian_stiffness.z > 5000.0) {
      valid = false;
      message += "+Invalid DoF Z stiffness";
    }
    // TODO: original values set by Calder were < 0.1 and > 300.0 - documentation states < 0.0 and > 300.0; why was it
    // set to 0.1?
    if (params.cartesian_stiffness.a < 0.0 || params.cartesian_stiffness.a > 300.0) {
      valid = false;
      message += "+Invalid DoF A stiffness";
    }
    if (params.cartesian_stiffness.b < 0.0 || params.cartesian_stiffness.b > 300.0) {
      valid = false;
      message += "+Invalid DoF B stiffness";
    }
    if (params.cartesian_stiffness.c < 0.0 || params.cartesian_stiffness.c > 300.0) {
      valid = false;
      message += "+Invalid DoF C stiffness";
    }
    if (params.nullspace_stiffness < 0.0) {
      valid = false;
      message += "+Invalid nullspace stiffness";
    }

    return {valid, message};
  }

  std::pair<bool, std::string> validateCartesianControlModeLimits(const msg::CartesianControlModeLimits& params) {
    bool valid = true;
    std::string message;

    // Path deviation
    if (params.max_path_deviation.x <= 0.0) {
      valid = false;
      message += "+Invalid DoF X max path deviation";
    }
    if (params.max_path_deviation.y <= 0.0) {
      valid = false;
      message += "+Invalid DoF Y max path deviation";
    }
    if (params.max_path_deviation.z <= 0.0) {
      valid = false;
      message += "+Invalid DoF Z max path deviation";
    }
    if (params.max_path_deviation.a <= 0.0) {
      valid = false;
      message += "+Invalid DoF A max path deviation";
    }
    if (params.max_path_deviation.b <= 0.0) {
      valid = false;
      message += "+Invalid DoF B max path deviation";
    }
    if (params.max_path_deviation.c <= 0.0) {
      valid = false;
      message += "+Invalid DoF C max path deviation";
    }

    // Cartesian velocity
    if (params.max_cartesian_velocity.x <= 0.0) {
      valid = false;
      message += "+Invalid DoF X max cartesian velocity";
    }
    if (params.max_cartesian_velocity.y <= 0.0) {
      valid = false;
      message += "+Invalid DoF Y max cartesian velocity";
    }
    if (params.max_cartesian_velocity.z <= 0.0) {
      valid = false;
      message += "+Invalid DoF Z max cartesian velocity";
    }
    if (params.max_cartesian_velocity.a <= 0.0) {
      valid = false;
      message += "+Invalid DoF A max cartesian velocity";
    }
    if (params.max_cartesian_velocity.b <= 0.0) {
      valid = false;
      message += "+Invalid DoF B max cartesian velocity";
    }
    if (params.max_cartesian_velocity.c <= 0.0) {
      valid = false;
      message += "+Invalid DoF C max cartesian velocity";
    }

    // Cartesian force
    if (params.max_control_force.x <= 0.0) {
      valid = false;
      message += "+Invalid DoF X max control force";
    }
    if (params.max_control_force.y <= 0.0) {
      valid = false;
      message += "+Invalid DoF Y max control force";
    }
    if (params.max_control_force.z <= 0.0) {
      valid = false;
      message += "+Invalid DoF Z max control force";
    }
    if (params.max_control_force.a <= 0.0) {
      valid = false;
      message += "+Invalid DoF A max control force";
    }
    if (params.max_control_force.b <= 0.0) {
      valid = false;
      message += "+Invalid DoF B max control force";
    }
    if (params.max_control_force.c <= 0.0) {
      valid = false;
      message += "+Invalid DoF C max control force";
    }

    return {valid, message};
  }

  std::pair<bool, std::string> validateControlMode(const msg::ControlModeParameters& params) {
    bool valid = true;
    std::string message;

    // Check the control mode itself
    if (params.control_mode.mode != msg::ControlMode::JOINT_POSITION &&
        params.control_mode.mode != msg::ControlMode::JOINT_IMPEDANCE &&
        params.control_mode.mode != msg::ControlMode::CARTESIAN_POSE &&
        params.control_mode.mode != msg::ControlMode::CARTESIAN_IMPEDANCE) {
      valid = false;
      message += "+Invalid control mode";
    }

    // Check each part of the control mode
    const auto valid_joint_impedance_params = validateJointImpedanceParams(params.joint_impedance_params);
    const auto valid_cartesian_impedance_params = validateCartesianImpedanceParams(params.cartesian_impedance_params);
    const auto valid_cartesian_control_mode_limits =
        validateCartesianControlModeLimits(params.cartesian_control_mode_limits);
    const auto valid_joint_path_execution_params = validateJointPathExecutionParams(params.joint_path_execution_params);
    const auto valid_cartesian_path_execution_params =
        validateCartesianPathExecutionParams(params.cartesian_path_execution_params);

    // Aggregate the results
    valid &= valid_joint_impedance_params.first && valid_cartesian_impedance_params.first &&
             valid_cartesian_control_mode_limits.first && valid_joint_path_execution_params.first &&
             valid_cartesian_path_execution_params.first;

    message += valid_joint_impedance_params.second + valid_cartesian_impedance_params.second +
               valid_cartesian_control_mode_limits.second + valid_joint_path_execution_params.second +
               valid_cartesian_path_execution_params.second;

    return {valid, message};
  }

  //// ROS Callbacks to get and set the control mode as service calls ////////////////////////////////////////////

  void setControlModeCallback(const srv::SetControlMode::Request::SharedPtr req,
                              srv::SetControlMode::Response::SharedPtr res) {
    std::optional<msg::ControlModeParameters> local_active_control_mode_copy;
    {
      std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
      local_active_control_mode_copy = active_control_mode_;
    }
    if (local_active_control_mode_copy) {
      const auto merged_command =
          mergeControlModeParameters(local_active_control_mode_copy.value(), req->new_control_mode);
      const auto validity_check = validateControlMode(merged_command);
      if (validity_check.first) {
        iiwa_ptr_->SendControlModeCommandMessage(merged_command);

        // Loop waiting for a matching control mode to be parsed
        bool control_mode_matches = false;

        const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

        do {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          end_time = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
          control_mode_matches = controlModeParamsEqual(merged_command, active_control_mode_.value());
        } while (!control_mode_matches &&
                 std::chrono::duration<double>(end_time - start_time).count() < set_control_mode_timeout_);

        // Check the results of the timeout
        if (control_mode_matches) {
          res->success = true;
          res->message = "Control mode set successfully";
        } else {
          res->success = false;
          res->message = "Control mode could not be set in Sunrise within the timeout window of " +
                         std::to_string(set_control_mode_timeout_);
        }
      } else {
        res->success = false;
        res->message = validity_check.second;
      }
    } else {
      res->success = false;
      res->message = "No initial control mode available from the controller";
    }
  }

  void getControlModeCallback(const srv::GetControlMode::Request::SharedPtr /*req*/,
                              srv::GetControlMode::Response::SharedPtr res) {
    std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
    res->has_active_control_mode = active_control_mode_.has_value();
    if (res->has_active_control_mode) {
      res->active_control_mode = active_control_mode_.value();
    }
  }

  /*
   * Callback function used by the LCM subsystem when a control_mode_status message is received. Caches the value
   * in active_control_mode_ for use by setControlModeCallback(...) and getControlModeCallback(...), as well as
   * publishes the message on the correct ROS topic
   */
  void controlModeStatusLCMCallback(const msg::ControlModeParameters& control_mode_status) {
    {
      std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
      if (!active_control_mode_) {
        RCLCPP_INFO(logger, "Initializing active_control_mode for the first time");
      }
      active_control_mode_ = control_mode_status;
    }
    control_mode_status_pub_->publish(control_mode_status);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Arm movement/control and feedback functionality
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //// Static helpers to check parameters for being in valid ranges for the motion command ///////////////////////

  std::pair<bool, std::string> validateCartesianPose(const geometry_msgs::msg::Pose& pose,
                                                     const std::string& frame) const {
    bool valid = true;
    std::string message;

    // Check to make sure the frame is correct
    if (frame != cartesian_control_frame_) {
      valid = false;
      message += "+Commanded cartesian pose has the wrong frame, " + frame + " given, " + cartesian_control_frame_ +
                 " expected";
    }

    // Check to make sure the quaternion is well-formed
    {
      const double quat_squared_norm =
          (pose.orientation.w * pose.orientation.w) + (pose.orientation.x * pose.orientation.x) +
          (pose.orientation.y * pose.orientation.y) + (pose.orientation.z * pose.orientation.z);
      const double error = std::fabs(1.0 - quat_squared_norm);
      if (error > 1e-6) {
        valid = false;
        message += "+Commanded cartesian pose quaternion is not normalized, squared norm = " +
                   std::to_string(quat_squared_norm);
      }
    }
    return {valid, message};
  }

  std::pair<bool, std::string> validateMotionCommand(const msg::MotionCommand& command) const {
    std::optional<msg::ControlModeParameters> active_control_mode_cached;
    {
      std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
      active_control_mode_cached = active_control_mode_;
    }
    if (active_control_mode_cached) {
      const uint8_t active_control_mode = active_control_mode_cached.value().control_mode.mode;
      const uint8_t command_motion_mode = command.control_mode.mode;

      // Note that this assumes the two messages use the same enums for each item, this is asserted in the constructor
      if (active_control_mode != command_motion_mode) {
        return std::make_pair(false, std::string("Active control mode does not match commanded control mode"));
      }

      switch (command_motion_mode) {
        case msg::ControlMode::JOINT_POSITION:
        case msg::ControlMode::JOINT_IMPEDANCE:
          return {true, ""};

        case msg::ControlMode::CARTESIAN_POSE:
        case msg::ControlMode::CARTESIAN_IMPEDANCE:
          return validateCartesianPose(command.cartesian_pose, command.header.frame_id);

        default:
          return {false, "Invalid commanded control mode. This should not be possible"};
      }
    } else {
      return {false, "No active control mode, cannot command motion"};
    }
  }

  /*
   * ROS callback to parse a arm motion command, and pass it along to the LCM subsystem
   */
  void motionCommandROSCallback(const msg::MotionCommand& command) {
    const auto validity_check_results = validateMotionCommand(command);
    if (validity_check_results.first) {
      iiwa_ptr_->SendMotionCommandMessage(command);
    } else {
      RCLCPP_ERROR_STREAM(logger, "Arm motion command failed validity checks: " << validity_check_results.second);
    }
  }

  /*
   * LCM callback used by the LCM subsystem when a LCM status message is recieved. Republishes the motion status
   * on the correct ROS topic
   */
  void motionStatusLCMCallback(const msg::MotionStatus& motion_status) { motion_status_pub_->publish(motion_status); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Gripper movement/control and feedback functionality
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //// Static helpers to check parameters for being in valid ranges for the gripper command //////////////////////

  static std::pair<bool, std::string> validateFingerCommand(const msg::Robotiq3FingerActuatorCommand& command) {
    bool valid = true;
    std::string message;
    if (command.position > 1.0 || command.position < 0.0) {
      valid = false;
      message += "+Invalid finger position";
    }

    if (command.force > 1.0 || command.force < 0.0) {
      valid = false;
      message += "+Invalid finger force";
    }

    if (command.speed > 1.0 || command.speed < 0.0) {
      valid = false;
      message += "+Invalid finger speed";
    }

    return {valid, message};
  }

  std::pair<bool, std::string> validateGripperCommand(const msg::Robotiq3FingerCommand& command) {
    const auto ac = validateFingerCommand(command.finger_a_command);
    const auto bc = validateFingerCommand(command.finger_b_command);
    const auto cc = validateFingerCommand(command.finger_c_command);
    const auto sc = validateFingerCommand(command.scissor_command);

    const bool valid = ac.first && bc.first && cc.first && sc.first;
    const std::string message = ac.second + bc.second + cc.second + sc.second;
    return {valid, message};
  }

  /*
   * ROS callback to parse a gripper motion command, and pass it along to the LCM subsystem
   */
  void gripperCommandROSCallback(const msg::Robotiq3FingerCommand& command) {
    const auto validity_check_results = validateGripperCommand(command);
    if (validity_check_results.first) {
      robotiq_ptr_->sendCommandMessage(command);
    } else {
      RCLCPP_WARN_STREAM(logger, "Gripper command failed validity checks: " << validity_check_results.second);
    }
  }

  /*
   * LCM callback used by the LCM subsystem when a LCM status message is recieved. Republishes the motion status
   * on the correct ROS topic
   */
  void gripperStatusLCMCallback(const msg::Robotiq3FingerStatus& gripper_status) {
    gripper_status_pub_->publish(gripper_status);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Internal helpers
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  static inline bool jointPathExecutionParamsIsDefault(const msg::JointPathExecutionParameters& params) {
    return (params.joint_relative_velocity == 0 && params.joint_relative_acceleration == 0 &&
            params.override_joint_acceleration == 0);
  }

  static inline bool cartesianPathExecutionParamsIsDefault(const msg::CartesianPathExecutionParameters& params) {
    return (params.max_velocity.x == 0 && params.max_velocity.y == 0 && params.max_velocity.z == 0 &&
            params.max_velocity.a == 0 && params.max_velocity.b == 0 && params.max_velocity.c == 0 &&
            params.max_acceleration.x == 0 && params.max_acceleration.y == 0 && params.max_acceleration.z == 0 &&
            params.max_acceleration.a == 0 && params.max_acceleration.b == 0 && params.max_acceleration.c == 0 &&
            params.max_nullspace_velocity == 0 && params.max_nullspace_acceleration == 0);
  }

  static inline bool jointImpedanceParamsIsDefault(const msg::JointImpedanceParameters& params) {
    return (params.joint_stiffness.joint_1 == 0 && params.joint_stiffness.joint_2 == 0 &&
            params.joint_stiffness.joint_3 == 0 && params.joint_stiffness.joint_4 == 0 &&
            params.joint_stiffness.joint_5 == 0 && params.joint_stiffness.joint_6 == 0 &&
            params.joint_stiffness.joint_7 == 0 && params.joint_damping.joint_1 == 0 &&
            params.joint_damping.joint_2 == 0 && params.joint_damping.joint_3 == 0 &&
            params.joint_damping.joint_4 == 0 && params.joint_damping.joint_5 == 0 &&
            params.joint_damping.joint_6 == 0 && params.joint_damping.joint_7 == 0);
  }

  static inline bool cartesianImpedanceParamsIsDefault(const msg::CartesianImpedanceParameters& params) {
    return (params.cartesian_stiffness.x == 0 && params.cartesian_stiffness.y == 0 &&
            params.cartesian_stiffness.z == 0 && params.cartesian_stiffness.a == 0 &&
            params.cartesian_stiffness.b == 0 && params.cartesian_stiffness.c == 0 && params.cartesian_damping.x == 0 &&
            params.cartesian_damping.y == 0 && params.cartesian_damping.z == 0 && params.cartesian_damping.a == 0 &&
            params.cartesian_damping.b == 0 && params.cartesian_damping.c == 0 && params.nullspace_stiffness == 0 &&
            params.nullspace_damping == 0);
  }

  static inline bool cartesianControlModeLimitsIsDefault(const msg::CartesianControlModeLimits& params) {
    return (params.max_path_deviation.x == 0 && params.max_path_deviation.y == 0 && params.max_path_deviation.z == 0 &&
            params.max_path_deviation.a == 0 && params.max_path_deviation.b == 0 && params.max_path_deviation.c == 0 &&
            params.max_cartesian_velocity.x == 0 && params.max_cartesian_velocity.y == 0 &&
            params.max_cartesian_velocity.z == 0 && params.max_cartesian_velocity.a == 0 &&
            params.max_cartesian_velocity.b == 0 && params.max_cartesian_velocity.c == 0 &&
            params.max_control_force.x == 0 && params.max_control_force.y == 0 && params.max_control_force.z == 0 &&
            params.max_control_force.a == 0 && params.max_control_force.b == 0 && params.max_control_force.c == 0 &&
            !params.stop_on_max_control_force);
  }

  static inline msg::ControlModeParameters mergeControlModeParameters(
      const msg::ControlModeParameters& active_control_mode, const msg::ControlModeParameters& new_control_mode) {
    /***************************************************************************************************************
    This function is a helper function for the callback function of setting a new control mode(setControlModeCallBack).
    It copies the parameters of the old control mode to the new one, and updates relevant parameters with theparameters
    of the new control mode.

    Parameters updated in each control mode:
    JOINT_POSITION: joint_path_execution_params
    CARTESIAN_POSE: cartesian_path_execution_params
    JOINT_IMPEDANCE: joint_impedance_params, joint_path_execution_params
    CARTESIAN_IMPEDANCE: cartesian_impedance_params, cartesian_control_mode_limits, cartesian_path_execution_params
    ***************************************************************************************************************/

    msg::ControlModeParameters merged_control_mode;
    // Copy the old over
    merged_control_mode.joint_path_execution_params = active_control_mode.joint_path_execution_params;
    merged_control_mode.joint_impedance_params = active_control_mode.joint_impedance_params;
    merged_control_mode.cartesian_impedance_params = active_control_mode.cartesian_impedance_params;
    merged_control_mode.cartesian_control_mode_limits = active_control_mode.cartesian_control_mode_limits;
    merged_control_mode.cartesian_path_execution_params = active_control_mode.cartesian_path_execution_params;
    // Copy manadatory members
    merged_control_mode.control_mode = new_control_mode.control_mode;
    // Copy mode-dependant members
    switch (new_control_mode.control_mode.mode) {
      case msg::ControlMode::JOINT_IMPEDANCE:
        merged_control_mode.joint_path_execution_params = new_control_mode.joint_path_execution_params;
        merged_control_mode.joint_impedance_params = new_control_mode.joint_impedance_params;

        if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params)) {
          RCLCPP_WARN(logger, "The cartesian impedance parameters are specified but ignored in JOINT_IMPEDANCE mode.");
        }
        if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits)) {
          RCLCPP_WARN(logger, "The cartesian control mode limits are specified but ignored in JOINT_IMPEDANCE mode.");
        }
        if (!cartesianPathExecutionParamsIsDefault(new_control_mode.cartesian_path_execution_params)) {
          RCLCPP_WARN(logger,
                      "The cartesian path execution parameters are specified but ignored in JOINT_IMPEDANCE mode.");
        }

        break;

      case msg::ControlMode::CARTESIAN_IMPEDANCE:
        merged_control_mode.cartesian_impedance_params = new_control_mode.cartesian_impedance_params;
        merged_control_mode.cartesian_control_mode_limits = new_control_mode.cartesian_control_mode_limits;
        merged_control_mode.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;

        if (!jointPathExecutionParamsIsDefault(new_control_mode.joint_path_execution_params)) {
          RCLCPP_WARN(logger,
                      "The joint path execution parameters are specified but ignored in CASRTESIAN_IMPEDANCE mode.");
        }
        if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params)) {
          RCLCPP_WARN(logger, "The joint impedance parameters are specified but ignored in CASRTESIAN_IMPEDANCE mode.");
        }

        break;

      case msg::ControlMode::JOINT_POSITION:
        // From the new
        merged_control_mode.joint_path_execution_params = new_control_mode.joint_path_execution_params;

        if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params)) {
          RCLCPP_WARN(logger, "The joint impedance parameters are specified but ignored in JOINT_POSITION mode.");
        }
        if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params)) {
          RCLCPP_WARN(logger, "The cartesian impedance parameters are specified but ignored in JOINT_POSITION mode.");
        }
        if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits)) {
          RCLCPP_WARN(logger, "The cartesian control mode limits are specified but ignored in JOINT_POSITION mode.");
        }
        if (!cartesianPathExecutionParamsIsDefault(new_control_mode.cartesian_path_execution_params)) {
          RCLCPP_WARN(logger,
                      "The cartesian path execution parameters are specified but ignored in JOINT_POSITION mode.");
        }

        break;

      case msg::ControlMode::CARTESIAN_POSE:
        // From the new
        merged_control_mode.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;

        if (!jointPathExecutionParamsIsDefault(new_control_mode.joint_path_execution_params)) {
          RCLCPP_WARN(logger, "The joint path execution parameters are specified but ignored in CARTESIAN_POSE mode.");
        }
        if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params)) {
          RCLCPP_WARN(logger, "The joint impedance parameters are specified but ignored in CARTESIAN_POSE mode.");
        }
        if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params)) {
          RCLCPP_WARN(logger, "The cartesian impedance parameters are specified but ignored in CARTESIAN_POSE mode.");
        }
        if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits)) {
          RCLCPP_WARN(logger, "The cartesian control mode limits are specified but ignored in CARTESIAN_POSE mode.");
        }

        break;

      default:
        RCLCPP_INFO_STREAM(logger, "Invalid control mode: " << new_control_mode.control_mode.mode << ".");
        assert(false);
    }

    return merged_control_mode;
  }

 private:
  constexpr static int LCM_HANDLE_TIMEOUT = 1;  // measured in milliseconds

  std::string cartesian_control_frame_;
  rclcpp::Publisher<msg::MotionStatus>::SharedPtr motion_status_pub_;
  rclcpp::Publisher<msg::ControlModeParameters>::SharedPtr control_mode_status_pub_;
  rclcpp::Publisher<msg::Robotiq3FingerStatus>::SharedPtr gripper_status_pub_;
  rclcpp::Subscription<msg::MotionCommand>::SharedPtr motion_command_sub_;
  rclcpp::Subscription<msg::Robotiq3FingerCommand>::SharedPtr gripper_command_sub_;
  rclcpp::Service<srv::SetControlMode>::SharedPtr set_control_mode_server_;
  rclcpp::Service<srv::GetControlMode>::SharedPtr get_control_mode_server_;

  mutable std::mutex control_mode_status_mutex_;
  std::optional<msg::ControlModeParameters> active_control_mode_;
  double set_control_mode_timeout_;

  std::shared_ptr<lcm::LCM> send_lcm_ptr_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
  std::unique_ptr<IiwaLcmBridge> iiwa_ptr_;
  std::unique_ptr<Robotiq3fLcmBridge> robotiq_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  // Start ROS
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<IiwaLcmBridgeNode>();
  rclcpp::spin(node);

  return 0;
}
