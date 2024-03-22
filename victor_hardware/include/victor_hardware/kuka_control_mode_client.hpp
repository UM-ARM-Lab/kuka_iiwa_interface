#include <lcm/lcm-cpp.hpp>
#include <memory>

#include <victor_lcm_interface/control_mode_parameters.hpp>
#include <victor_hardware/lcm_listener.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class KukaControlModeClient {
 public:
  KukaControlModeClient(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::string const& recv_provider, std::string const& send_provider);

  bool updateControlMode(victor_lcm_interface::control_mode_parameters const& new_control_mode);

  uint8_t getControlMode() const;

 private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
  std::shared_ptr<lcm::LCM> send_lcm_ptr_;

  // listener
  std::unique_ptr<LcmListener<victor_lcm_interface::control_mode_parameters>> control_mode_listener_;
};
