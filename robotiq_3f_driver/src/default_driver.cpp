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

#include <serial/serial.h>

#include <cmath>
#include <iostream>

#include <robotiq_3f_driver/default_driver.hpp>

#include <robotiq_3f_driver/crc_utils.hpp>
#include <robotiq_3f_driver/data_utils.hpp>
#include <robotiq_3f_driver/default_driver_utils.hpp>
#include <robotiq_3f_driver/driver_exception.hpp>

#include <rclcpp/logging.hpp>
#include <type_traits>

// +-----+-------------------------------+-------------------------------+
// |     | Gripper Input Registers       | Gripper Output Registers      |
// +-----+-------------------------------+-------------------------------+
// | Idx | Address | Function            | Address | Function            |
// +-----+---------+---------------------+---------+---------------------+
// |   0 | 0x03E8  | Action Request      | 0x07D0  | Gripper Status      |
// |   1 |         | Gripper Options     |         | Object Detection    |
// |   2 | 0x03E9  | Empty               | 0x07D1  | Fault status        |
// |   3 |         | Finger A Position   |         | Finger A Req Echo   |
// |   4 | 0x03EA  | Finger A Speed      | 0x07D2  | Finger A Position   |
// |   5 |         | Finger A Force      |         | Finger A Current    |
// |   6 | 0x03EB  | Finger B Position   | 0x07D3  | Finger B Req Echo   |
// |   7 |         | Finger B Speed      |         | Finger B Position   |
// |   8 | 0x03EC  | Finger B Force      | 0x07D4  | Finger B Current    |
// |   9 |         | Finger C Position   |         | Finger C Req Echo   |
// |  10 | 0x03ED  | Finger C Speed      | 0x07D5  | Finger C Position   |
// |  11 |         | Finger C Force      |         | Finger C Current    |
// |  12 | 0x03EE  | Scissor Position    | 0x07D6  | Scissor Req Echo    |
// |  13 |         | Scissor Speed       |         | Scissor Position    |
// |  14 | 0x03EF  | Scissor Force       | 0x07D7  | Scissor Current     |
// |  15 |         | Reserved            |         | Reserved            |
//       +---------+---------------------+---------+---------------------+

namespace robotiq_3f_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriver");

// If the gripper connection is not stable we may want to try sending the command again.
constexpr auto kMaxRetries = 5;

constexpr uint16_t kActionRequestRegisterAddress = 0x03E8;
constexpr uint16_t kGripperStatusRegister = 0x07D0;

constexpr size_t kActivateResponseSize = 8;
constexpr size_t kDeactivateResponseSize = 8;
constexpr size_t kNumModBusRegisters = 8;
constexpr size_t kResponseSizeHeaderSize = 5;

DefaultDriver::DefaultDriver(std::unique_ptr<Serial> serial) : serial_{ std::move(serial) }
{
}

std::vector<uint8_t> DefaultDriver::send(const std::vector<uint8_t>& request, size_t response_size) const
{
  std::stringstream req_ss;
  std::vector<uint8_t> response;
  response.reserve(response_size);

  int retry_count = 0;
  while (retry_count < kMaxRetries)
  {
    try
    {
      auto const t0 = std::chrono::high_resolution_clock::now();
      serial_->write(request);
      response = serial_->read(response_size);
      auto const t1 = std::chrono::high_resolution_clock::now();
      auto const dt = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
      RCLCPP_DEBUG_STREAM(kLogger, "serial dt: " << dt << " us");

      break;
    }
    catch (const serial::IOException& e)
    {
      RCLCPP_DEBUG(kLogger, "Resending the command because the previous attempt (%d of %d) failed: %s", retry_count + 1,
                   kMaxRetries, e.what());
      retry_count++;
    }
  }

  if (retry_count == kMaxRetries)
  {
    RCLCPP_ERROR(kLogger, "Reached maximum retries. Operation failed.");
    return {};
  }

  return response;
}

void DefaultDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
  RCLCPP_INFO(kLogger, "slave_address set to: %d", slave_address);
}

bool DefaultDriver::connect()
{
  serial_->open();
  return serial_->is_open();
}

void DefaultDriver::disconnect()
{
  serial_->close();
}

void DefaultDriver::activate()
{
  RCLCPP_INFO(kLogger, "Activate...");

  // built up the Action request register (see 4.4)
  uint8_t action_request_register = 0b00000000;
  default_driver_utils::set_gripper_activation(action_request_register, GripperActivationAction::ACTIVE);
  // Ensure no accidental motion, since we don't know what values the command registers might have
  default_driver_utils::set_go_to(action_request_register, GoTo::STOP);

  build_request_and_send({ action_request_register, 0x00 }, kActivateResponseSize);

  wait_until_activated();
}

void DefaultDriver::clear_faults()
{
  RCLCPP_INFO(kLogger, "Clearing faults...");

  build_request_and_send({ 0x00, 0x00 }, 8);
}

void DefaultDriver::build_request_and_send(std::vector<uint8_t> regs, size_t const response_size)
{
  // check that we don't send so much data that the size won't fit in uint8_t
  if (regs.size() > std::numeric_limits<uint8_t>::max())
  {
    throw DriverException{ "Too many registers to send_independent_control_command!" };
  }

  // since there are two robotiq registers per modbus register, we must always send a even number of robotiq registers
  if (regs.size() % 2 != 0)
  {
    throw DriverException{ "Must send an even number of robotiq registers!" };
  }

  uint8_t const num_robotiq_registers = regs.size();
  uint8_t const num_modbus_registers = num_robotiq_registers / 2;
  std::vector<uint8_t> request = {
    slave_address_, static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress), data_utils::get_lsb(kActionRequestRegisterAddress),
    data_utils::get_msb(num_modbus_registers), data_utils::get_lsb(num_modbus_registers),
    // Number of bytes to send_independent_control_command, which is number of robotiq registers since they're one byte each.
    num_robotiq_registers
  };
  request.insert(request.end(), regs.begin(), regs.end());
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, response_size);
  if (response.empty())
  {
    throw DriverException{ "Failed to activate the gripper." };
  }
}

void DefaultDriver::wait_until_activated()
{  // now wait until the activation is complete
  while (true)
  {
    auto status = get_full_status();
    if (status.gripper_status == GripperStatus::ACTIVATED)
    {
      break;
    }
  }
}

void DefaultDriver::deactivate()
{
  RCLCPP_INFO(kLogger, "Deactivate...");

  // See 4.7.6 Modbus RTU Example, step 1: Activation request in the manual.
  build_request_and_send({ 0x00, 0x00 }, kDeactivateResponseSize);
}

FullGripperStatus DefaultDriver::get_full_status()
{
  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::ReadInputRegisters),
    data_utils::get_msb(kGripperStatusRegister),
    data_utils::get_lsb(kGripperStatusRegister),
    data_utils::get_msb(kNumModBusRegisters),
    data_utils::get_lsb(kNumModBusRegisters),
  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, kResponseSizeHeaderSize + 2 * kNumModBusRegisters);
  if (response.empty())
  {
    throw DriverException{ "Failed to read the status." };
  }

  // The content of the requested registers starts from byte 3.
  // byte 3 contains the gripper status, which includes gACT through gSTA.
  // byte 4 contains the object status, which is gDTA through gDTS.
  // byte 5 contains the fault status (gFLT).
  FullGripperStatus status{};
  status.activation_status = default_driver_utils::get_gripper_activation_status(response[3]);
  status.mode_status = default_driver_utils::get_grasping_mode(response[3]);
  status.go_to_status = default_driver_utils::get_go_to_status(response[3]);
  status.gripper_status = default_driver_utils::get_gripper_status(response[3]);
  status.motion_status = default_driver_utils::get_motion_status(response[3]);
  status.finger_a_object_detection_status = default_driver_utils::get_finger_a_object_status(response[4]);
  status.finger_b_object_detection_status = default_driver_utils::get_finger_b_object_status(response[4]);
  status.finger_c_object_detection_status = default_driver_utils::get_finger_c_object_status(response[4]);
  status.scissor_object_detection_status = default_driver_utils::get_scissor_object_status(response[4]);
  status.fault_status = default_driver_utils::get_gripper_fault_status(response[5]);
  status.finger_a_position_cmd_echo = response[6];
  status.finger_a_position = response[7];
  status.finger_a_current = response[8];
  status.finger_b_position_cmd_echo = response[9];
  status.finger_b_position = response[10];
  status.finger_b_current = response[11];
  status.finger_c_position_cmd_echo = response[12];
  status.finger_c_position = response[13];
  status.finger_c_current = response[14];
  status.scissor_position_cmd_echo = response[15];
  status.scissor_position = response[16];
  status.scissor_current = response[17];

  return status;
}

void DefaultDriver::send_independent_control_command(IndependentControlCommand const& cmd)
{
  // set all the position, velocity, and force registers, as well as the GO_TO bits
  // the ACT bit must also still be set
  uint8_t action_request_register = 0b00000000;
  default_driver_utils::set_gripper_activation(action_request_register, GripperActivationAction::ACTIVE);
  // Ensure no accidental motion, since we don't know what values the command registers might have
  default_driver_utils::set_go_to(action_request_register, GoTo::GO_TO);

  uint8_t gripper_options_register = 0b00000000;
  default_driver_utils::set_individual_control_mode(gripper_options_register, true);
  default_driver_utils::set_individual_scissor_control_mode(gripper_options_register, true);

  build_request_and_send(
      {
          action_request_register, gripper_options_register,
          0x00,  // This register must always be empty
          default_driver_utils::double_to_uint8(cmd.finger_a_position),
          default_driver_utils::double_to_uint8(cmd.finger_a_velocity),
          default_driver_utils::double_to_uint8(cmd.finger_a_force),
          default_driver_utils::double_to_uint8(cmd.finger_b_position),
          default_driver_utils::double_to_uint8(cmd.finger_b_velocity),
          default_driver_utils::double_to_uint8(cmd.finger_b_force),
          default_driver_utils::double_to_uint8(cmd.finger_c_position),
          default_driver_utils::double_to_uint8(cmd.finger_c_velocity),
          default_driver_utils::double_to_uint8(cmd.finger_c_force),
          default_driver_utils::double_to_uint8(cmd.scissor_position),
          default_driver_utils::double_to_uint8(cmd.scissor_velocity),
          default_driver_utils::double_to_uint8(cmd.scissor_force),
          0x00,  // Reserved.
      },
      8);

  // NOTE: Do I need to check the response?
}

void DefaultDriver::send_simple_control_command(GraspingMode const& mode, double position, double velocity, double force)
{
  uint8_t action_request_register = 0b00000000;
  default_driver_utils::set_gripper_activation(action_request_register, GripperActivationAction::ACTIVE);
  default_driver_utils::set_go_to(action_request_register, GoTo::GO_TO);
  default_driver_utils::set_grasping_mode(action_request_register, mode);

  uint8_t gripper_options_register = 0b00000000;
  default_driver_utils::set_individual_control_mode(gripper_options_register, false);
  default_driver_utils::set_individual_scissor_control_mode(gripper_options_register, false);

  build_request_and_send(
      {
          action_request_register,
          gripper_options_register,
          0x00,  // This register must always be empty
          default_driver_utils::double_to_uint8(position),
          default_driver_utils::double_to_uint8(velocity),
          default_driver_utils::double_to_uint8(force),
          0x00,  // The rest of the registers must be empty when using Simple Control Mode
          0x00,
          0x00,
          0x00,
          0x00,
          0x00,
          0x00,
          0x00,
          0x00,
          0x00,
      },
      8);
}

DefaultDriver::~DefaultDriver()
{
  DefaultDriver::deactivate();
  DefaultDriver::disconnect();
}

}  // namespace robotiq_3f_driver
