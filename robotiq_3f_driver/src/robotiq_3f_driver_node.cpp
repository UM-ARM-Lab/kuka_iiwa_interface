// Copyright (c) 2023 Peter Mitrano
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <robotiq_3f_driver/default_driver.hpp>
#include <robotiq_3f_driver/default_serial.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <robotiq_3f_driver/default_driver_utils.hpp>
#include <robotiq_3f_interfaces/srv/change_grasping_mode.hpp>
#include <robotiq_3f_interfaces/msg/status.hpp>
#include <robotiq_3f_interfaces/msg/simple_control_command.hpp>
#include <robotiq_3f_interfaces/msg/independent_control_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robotiq_3f_transmission_plugins/individual_control_transmission.hpp>
#include <control_msgs/action/gripper_command.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace robotiq_3f_driver;

auto const kLogger = rclcpp::get_logger("Robotiq3fDriverNode");

auto mode_msg_to_mode(robotiq_3f_interfaces::msg::GraspingMode mode_msg)
{
  switch (mode_msg.mode)
  {
    case robotiq_3f_interfaces::msg::GraspingMode::BASIC:
      return GraspingMode::BASIC;
    case robotiq_3f_interfaces::msg::GraspingMode::PINCH:
      return GraspingMode::PINCH;
    case robotiq_3f_interfaces::msg::GraspingMode::WIDE:
      return GraspingMode::WIDE;
    case robotiq_3f_interfaces::msg::GraspingMode::SCISSOR:
      return GraspingMode::SCISSOR;
    default:
      throw std::runtime_error("Unknown grasping mode");
  }
}

auto mode_to_mode_msg(GraspingMode mode)
{
  switch (mode)
  {
    case GraspingMode::BASIC:
      return robotiq_3f_interfaces::msg::GraspingMode::BASIC;
    case GraspingMode::PINCH:
      return robotiq_3f_interfaces::msg::GraspingMode::PINCH;
    case GraspingMode::WIDE:
      return robotiq_3f_interfaces::msg::GraspingMode::WIDE;
    case GraspingMode::SCISSOR:
      return robotiq_3f_interfaces::msg::GraspingMode::SCISSOR;
    default:
      throw std::runtime_error("Unknown grasping mode");
  }
}

auto obj_to_obj_msg(ObjectDetectionStatus obj)
{
  switch (obj)
  {
    case ObjectDetectionStatus::MOVING:
      return robotiq_3f_interfaces::msg::ObjectDetectionStatus::MOVING;
    case ObjectDetectionStatus::OBJECT_DETECTED_OPENING:
      return robotiq_3f_interfaces::msg::ObjectDetectionStatus::OBJECT_DETECTED_OPENING;
    case ObjectDetectionStatus::OBJECT_DETECTED_CLOSING:
      return robotiq_3f_interfaces::msg::ObjectDetectionStatus::OBJECT_DETECTED_CLOSING;
    case ObjectDetectionStatus::AT_REQUESTED_POSITION:
      return robotiq_3f_interfaces::msg::ObjectDetectionStatus::AT_REQUESTED_POSITION;
    default:
      throw std::runtime_error("Unknown object detection status");
  }
}

class ROSDriver : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommand>;

  explicit ROSDriver() : Node("robotiq_3f_driver")
  {
    // ROS params
    declare_parameter<double>("speed_for_gripper_action", 1.0);
    declare_parameter<double>("pub_period_seconds", 0.01);

    speed_for_gripper_action_ = get_parameter("speed_for_gripper_action").as_double();
    pub_period_seconds_ = get_parameter("pub_period_seconds").as_double();

    auto serial = std::make_unique<DefaultSerial>();
    serial->set_port("/dev/ttyUSB1");
    serial->set_baudrate(115200);
    serial->set_timeout(500ms);

    driver_ = std::make_unique<DefaultDriver>(std::move(serial));
    driver_->set_slave_address(0x09);

    driver_->connect();
    driver_->clear_faults();
    driver_->activate();

    // Set up the ROS API
    // Things with callbacks that are very quick can all go on the default callback group
    status_pub_ = create_publisher<robotiq_3f_interfaces::msg::Status>("status", 10);
    rclcpp::QoS js_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", js_qos);

    independent_control_command_sub_ = create_subscription<robotiq_3f_interfaces::msg::IndependentControlCommand>(
        "independent_control_command", 10,
        [&](const std::shared_ptr<robotiq_3f_interfaces::msg::IndependentControlCommand> msg) {
          IndependentControlCommand cmd;
          cmd.finger_a_position = msg->finger_a_position;
          cmd.finger_b_position = msg->finger_b_position;
          cmd.finger_c_position = msg->finger_c_position;
          cmd.scissor_position = msg->scissor_position;
          cmd.finger_a_velocity = msg->finger_a_velocity;
          cmd.finger_b_velocity = msg->finger_b_velocity;
          cmd.finger_c_velocity = msg->finger_c_velocity;
          cmd.scissor_velocity = msg->scissor_velocity;
          cmd.finger_a_force = msg->finger_a_force;
          cmd.finger_b_force = msg->finger_b_force;
          cmd.finger_c_force = msg->finger_c_force;
          cmd.scissor_force = msg->scissor_force;

          {
            std::scoped_lock lock(driver_mutex_);
            driver_->send_independent_control_command(cmd);
          }
        });

    simple_control_command_sub_ = create_subscription<robotiq_3f_interfaces::msg::SimpleControlCommand>(
        "simple_control_command", 10, [&](const std::shared_ptr<robotiq_3f_interfaces::msg::SimpleControlCommand> msg) {
          std::scoped_lock lock(driver_mutex_);
          driver_->send_simple_control_command(latest_mode_, msg->position, msg->velocity, msg->force);
        });

    // But slow things get their own callback groups
    change_grasping_mode_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    change_grasping_mode_service_ = create_service<robotiq_3f_interfaces::srv::ChangeGraspingMode>(
        "change_grasping_mode", std::bind(&ROSDriver::change_grasping_mode, this, _1, _2),
        rmw_qos_profile_services_default, change_grasping_mode_cb_group_);

    gripper_command_action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gripper_command_action_ = rclcpp_action::create_server<GripperCommand>(
        this, "gripper_cmd",  // NOTE: this must match the string in the moveit plugin allocator
        [&](rclcpp_action::GoalUUID const& uuid, GripperCommand::Goal::ConstSharedPtr goal) {
          (void)uuid;
          // check that the max effort is below 1
          if (goal->command.max_effort > 1.0)
          {
            RCLCPP_ERROR(kLogger, "Max effort should be between 0 (min effort) and 1.0 (max effort)");
            return rclcpp_action::GoalResponse::REJECT;
          }
          RCLCPP_INFO(kLogger, "Accepting goal");
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // Accept all cancellation requests
        [&](const std::shared_ptr<GoalHandle>& goal_handle) {
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        // If we've accepted a goal, start a thread to process it, so we don't block the ROS executor
        [&](const std::shared_ptr<GoalHandle> goal_handle) {
          // NOTE: std::bind looks pretty clean here compared to the lambda. I tried some version of the lambda,
          // but they caused things to crash, so I'm sticking with this for now.
          std::thread{ std::bind(&ROSDriver::execute_gripper_command, this, _1), goal_handle }.detach();
        },
        rcl_action_server_get_default_options(), gripper_command_action_cb_group_);

    // Create a timer callback at 100HZ to publish the states
    timer_ = create_wall_timer(std::chrono::duration<double>(pub_period_seconds_), [&]() {
      FullGripperStatus status{};
      {
        std::scoped_lock lock(driver_mutex_);
        status = driver_->get_full_status();
      }

      robotiq_3f_interfaces::msg::Status status_msg;
      status_msg.mode.mode = mode_to_mode_msg(latest_mode_);
      status_msg.finger_a_position = status.finger_a_position;
      status_msg.finger_b_position = status.finger_b_position;
      status_msg.finger_c_position = status.finger_c_position;
      status_msg.scissor_position = status.scissor_position;
      status_msg.finger_a_current = status.finger_a_current;
      status_msg.finger_b_current = status.finger_b_current;
      status_msg.finger_c_current = status.finger_c_current;
      status_msg.scissor_current = status.scissor_current;
      status_msg.finger_a_cmd_echo = status.finger_a_position_cmd_echo;
      status_msg.finger_b_cmd_echo = status.finger_b_position_cmd_echo;
      status_msg.finger_c_cmd_echo = status.finger_c_position_cmd_echo;
      status_msg.scissor_cmd_echo = status.scissor_position_cmd_echo;
      status_msg.finger_a_object_detection.status = obj_to_obj_msg(status.finger_a_object_detection_status);
      status_msg.finger_b_object_detection.status = obj_to_obj_msg(status.finger_b_object_detection_status);
      status_msg.finger_c_object_detection.status = obj_to_obj_msg(status.finger_c_object_detection_status);
      status_msg.scissor_object_detection.status = obj_to_obj_msg(status.scissor_object_detection_status);

      status_pub_->publish(status_msg);

      auto const finger_a_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(status.finger_a_position);
      auto const finger_b_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(status.finger_b_position);
      auto const finger_c_thetas = robotiq_3f_transmission_plugins::get_finger_thetas(status.finger_c_position);
      auto const palm_finger = robotiq_3f_transmission_plugins::get_palm_finger_pos(status.scissor_position);

      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = now();
      joint_state_msg.name = { "finger_a_joint_1", "finger_a_joint_2",    "finger_a_joint_3",   "finger_b_joint_1",
                               "finger_b_joint_2", "finger_b_joint_3",    "finger_c_joint_1",   "finger_c_joint_2",
                               "finger_c_joint_3", "palm_finger_b_joint", "palm_finger_c_joint" };
      joint_state_msg.position = { finger_a_thetas[0], finger_a_thetas[1], finger_a_thetas[2], finger_b_thetas[0],
                                   finger_b_thetas[1], finger_b_thetas[2], finger_c_thetas[0], finger_c_thetas[1],
                                   finger_c_thetas[2], palm_finger,        -palm_finger };

      joint_state_pub_->publish(joint_state_msg);
    });
  }

  /**
   * This is a service server that will not block the ROS executor.
   * @param req
   * @param res
   */
  void change_grasping_mode(robotiq_3f_interfaces::srv::ChangeGraspingMode::Request::ConstSharedPtr req,
                            robotiq_3f_interfaces::srv::ChangeGraspingMode::Response::SharedPtr res)
  {
    latest_mode_ = mode_msg_to_mode(req->mode);
    RCLCPP_DEBUG(kLogger, "Changing grasping mode");
    {
      std::scoped_lock lock(driver_mutex_);
      driver_->send_simple_control_command(latest_mode_, 0, 0, 0);
    }

    // Wait until the motion status said we've finished changing modes.
    // and yes it really can take a long time!
    auto const success = wait_until_reached(30.0);

    res->success = success;
  }

  void execute_gripper_command(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto const cmd = goal_handle->get_goal()->command;
    RCLCPP_DEBUG_STREAM(kLogger, "Executing goal: " << cmd.position << ", " << speed_for_gripper_action_ << ", "
                                                    << cmd.max_effort << ", "
                                                    << default_driver_utils::grasping_mode_to_string(latest_mode_));
    {
      std::scoped_lock lock(driver_mutex_);
      driver_->send_simple_control_command(latest_mode_, cmd.position, speed_for_gripper_action_, cmd.max_effort);
    }

    auto result = std::make_shared<GripperCommand::Result>();
    auto const success = wait_until_reached(30.0);
    result->reached_goal = success;

    goal_handle->succeed(result);
  }

  bool wait_until_reached(double timeout)
  {
    auto const t0 = std::chrono::steady_clock::now();
    while (true)
    {
      FullGripperStatus status{};
      {
        std::scoped_lock lock(driver_mutex_);
        status = driver_->get_full_status();
      }
      auto const motion_stopped = status.motion_status == MotionStatus::STOPPED_REACHED;
      auto const not_changing_modes = status.gripper_status != GripperStatus::MODE_CHANGE_IN_PROGRESS;
      if (motion_stopped && not_changing_modes)
      {
        return true;
      }

      auto const t1 = std::chrono::steady_clock::now();
      auto const dt = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
      if (dt > timeout)
      {
        return false;
      }

      // let other threads/callbacks run
      std::this_thread::yield();
      std::this_thread::sleep_for(100ms);
    }
  }

private:
  // We need to prevent various callbacks from running at the same time
  std::mutex driver_mutex_;

  std::unique_ptr<DefaultDriver> driver_;
  double speed_for_gripper_action_;
  double pub_period_seconds_;

  // These must be listed after the driver to ensure they are destroyed first, and all callbacks stop before
  // driver is destroyed.
  rclcpp::Service<robotiq_3f_interfaces::srv::ChangeGraspingMode>::SharedPtr change_grasping_mode_service_;
  rclcpp::Publisher<robotiq_3f_interfaces::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<robotiq_3f_interfaces::msg::IndependentControlCommand>::SharedPtr independent_control_command_sub_;
  rclcpp::Subscription<robotiq_3f_interfaces::msg::SimpleControlCommand>::SharedPtr simple_control_command_sub_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_action_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::CallbackGroup::SharedPtr change_grasping_mode_cb_group_;
  rclcpp::CallbackGroup::SharedPtr gripper_command_action_cb_group_;

  GraspingMode latest_mode_ = GraspingMode::BASIC;
};

int main(int argc, const char** argv)
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Instantiate the ROS Wrapper
  auto ros_driver = std::make_shared<ROSDriver>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ros_driver);
  executor.spin();
}
