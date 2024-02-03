#include <atomic>
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>

using namespace moveit_servo;

namespace
{
constexpr auto K_BASE_FRAME = "victor_left_arm_link0";
constexpr auto K_TIP_FRAME = "victor_left_arm_link7";
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto demo_node = std::make_shared<rclcpp::Node>("victor_servo");
  moveit::setNodeLoggerName(demo_node->get_name());

  // Get the servo parameters.
  const std::string param_namespace = "moveit_servo";
  const auto servo_param_listener = std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const auto servo_params = servo_param_listener->get_params();

  // The publisher to send trajectory message to the robot controller.
  auto trajectory_outgoing_cmd_pub = demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());

  // Create the servo object
  const auto planning_scene_monitor = createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo(demo_node, servo_param_listener, planning_scene_monitor);

  // Helper function to get the current pose of a specified frame.
  const auto get_current_pose = [](const std::string& target_frame, const moveit::core::RobotStatePtr& robot_state) {
    return robot_state->getGlobalLinkTransform(target_frame);
  };

  // Wait for some time, so that the planning scene is loaded in rviz.
  // This is just for convenience, should not be used for sync in real application.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Get the robot state and joint model group info.
  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const auto joint_model_group = robot_state->getJointModelGroup(servo_params.move_group_name);

  // Set the command type for servo.
  servo.setCommandType(CommandType::POSE);

  // The dynamically updated target pose.
  PoseCommand target_pose;
  target_pose.frame_id = K_BASE_FRAME;

  // Initializing the target pose as end effector pose, this can be any pose.
  target_pose.pose = get_current_pose(K_TIP_FRAME, robot_state);

  // servo loop will exit upon reaching this pose.
  Eigen::Isometry3d terminal_pose = target_pose.pose;
  terminal_pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  terminal_pose.translate(Eigen::Vector3d(0.0, 0.0, -0.1));

  // The target pose (frame being tracked) moves by this step size each iteration.
  Eigen::Vector3d linear_step_size{ 0.0, 0.0, -0.001 };
  Eigen::AngleAxisd angular_step_size(0.01, Eigen::Vector3d::UnitZ());

  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());

  rclcpp::WallRate servo_rate(1 / servo_params.publish_period);

  // create command queue to build trajectory message and add current robot state
  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState();
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

  bool satisfies_linear_tolerance = false;
  bool satisfies_angular_tolerance = false;
  bool stop_tracking = false;
  while (!stop_tracking && rclcpp::ok())
  {
    // check if goal reached
    target_pose.pose = get_current_pose(K_TIP_FRAME, robot_state);
    satisfies_linear_tolerance |= target_pose.pose.translation().isApprox(terminal_pose.translation(),
                                                                          servo_params.pose_tracking.linear_tolerance);
    satisfies_angular_tolerance |=
        target_pose.pose.rotation().isApprox(terminal_pose.rotation(), servo_params.pose_tracking.angular_tolerance);
    stop_tracking = satisfies_linear_tolerance && satisfies_angular_tolerance;

    // Dynamically update the target pose.
    if (!satisfies_linear_tolerance)
    {
      target_pose.pose.translate(linear_step_size);
    }
    if (!satisfies_angular_tolerance)
    {
      target_pose.pose.rotate(angular_step_size);
    }

    // get next servo command
    KinematicState joint_state = servo.getNextJointState(robot_state, target_pose);
    StatusCode status = servo.getStatus();
    if (status != StatusCode::INVALID)
    {
      updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());
      if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
      {
        trajectory_outgoing_cmd_pub->publish(msg.value());
      }
      if (!joint_cmd_rolling_window.empty())
      {
        robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
        robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
      }
    }

    servo_rate.sleep();
  }

  RCLCPP_INFO_STREAM(demo_node->get_logger(), "REACHED : " << stop_tracking);
  RCLCPP_INFO(demo_node->get_logger(), "Exiting demo.");
  rclcpp::shutdown();
}
