// An executor that calls spin in a separate thread

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace victor_hardware {

class AsyncExecutor : public rclcpp::executors::MultiThreadedExecutor {
 public:
  // Create an instance and start the internal thread
  AsyncExecutor();
  AsyncExecutor(const AsyncExecutor&) = delete;
  AsyncExecutor(AsyncExecutor&&) = delete;

  AsyncExecutor& operator=(const AsyncExecutor&) = delete;
  AsyncExecutor& operator=(AsyncExecutor&&) = delete;

  // Stops the internal executor and joins with the internal thread
  ~AsyncExecutor() override;

 private:
  std::thread executor_spin_;

  // Executor thread starts spining the multithreadedExecutor
  void run();

  // Cancel any spinning ROS executor
  void shutdown();
};
}  // namespace victor_hardware
