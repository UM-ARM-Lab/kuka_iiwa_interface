#pragma once

#include <cassert>
#include <vector>
#include <victor_hardware_interfaces/msg/joint_value_quantity.hpp>
#include <victor_lcm_interface/joint_value_quantity.hpp>
#include <rclcpp/logging.hpp>

auto const logger = rclcpp::get_logger("victor_lcm_bridge");

namespace victor_utils {
namespace msg = victor_hardware_interfaces::msg;

static msg::JointValueQuantity vectorToJvq(const std::vector<double> &v) {
  assert(v.size() == 7);

  msg::JointValueQuantity jvq;
  jvq = msg::JointValueQuantity();
  jvq.joint_1 = v[0];
  jvq.joint_2 = v[1];
  jvq.joint_3 = v[2];
  jvq.joint_4 = v[3];
  jvq.joint_5 = v[4];
  jvq.joint_6 = v[5];
  jvq.joint_7 = v[6];
  return jvq;
};

static std::vector<double> jvqToVector(const msg::JointValueQuantity &jvq) {
  std::vector<double> v(7);
  v[0] = jvq.joint_1;
  v[1] = jvq.joint_2;
  v[2] = jvq.joint_3;
  v[3] = jvq.joint_4;
  v[4] = jvq.joint_5;
  v[5] = jvq.joint_6;
  v[6] = jvq.joint_7;
  return v;
};

}  // namespace victor_utils