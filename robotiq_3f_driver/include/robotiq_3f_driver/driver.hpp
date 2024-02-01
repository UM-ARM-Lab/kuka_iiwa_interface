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

#pragma once

#include <chrono>
#include <string>

namespace robotiq_3f_driver
{

// These structs give us a friendlier interface to the registers of the gripper.
enum class GripperActivationAction
{
  RESET,
  ACTIVE
};
// They are the same, but the names are clearer this way
enum class GripperActivationStatus
{
  INACTIVE,
  ACTIVE
};

enum class GraspingMode
{
  BASIC,
  PINCH,
  WIDE,
  SCISSOR
};

enum class GoTo
{
  STOP,
  GO_TO
};

enum class ActionStatus
{
  STOPPED,
  MOVING
};

enum class MotionStatus  // gSTA
{
  IN_MOTION,
  STOPPED_ONE_OR_TWO_UNREACHED,
  STOPPED_THREE_UNREACHED,
  STOPPED_REACHED
};

enum class ObjectDetectionStatus : uint8_t
{
  MOVING,
  OBJECT_DETECTED_OPENING,
  OBJECT_DETECTED_CLOSING,
  AT_REQUESTED_POSITION

};

enum class GripperStatus
{
  RESET,
  ACTIVATION_IN_PROGRESS,
  MODE_CHANGE_IN_PROGRESS,
  ACTIVATED
};

enum class GripperFaultStatus
{
  // priority faults
  ACTION_DELAYED_REACTIVATION_NEEDED,
  ACTION_DELAYED_MODE_CHANGE_NEEDED,
  ACTIVATION_BIT_NOT_SET,
  // minor faults
  NOT_READY,
  MODE_CHANGE_SCISSOR_MOTION_ERROR,
  AUTO_RELEASE_IN_PROGRESS,
  // major faults
  ACTIVATION_FAULT,
  MODE_CHANGE_FAULT,
  AUTOMATIC_RELEASE_FAULT,
};

struct FullGripperStatus
{
  GripperActivationStatus activation_status;  // an echo of the activation command
  GraspingMode mode_status;
  GoTo go_to_status;
  GripperStatus gripper_status;
  MotionStatus motion_status;
  GripperFaultStatus fault_status;

  uint8_t finger_a_position_cmd_echo;
  uint8_t finger_a_position;
  uint8_t finger_a_current;
  uint8_t finger_b_position_cmd_echo;
  uint8_t finger_b_position;
  uint8_t finger_b_current;
  uint8_t finger_c_position_cmd_echo;
  uint8_t finger_c_position;
  uint8_t finger_c_current;
  uint8_t scissor_position_cmd_echo;
  uint8_t scissor_position;
  uint8_t scissor_current;

  // Object detection status
  ObjectDetectionStatus finger_a_object_detection_status;
  ObjectDetectionStatus finger_b_object_detection_status;
  ObjectDetectionStatus finger_c_object_detection_status;
  ObjectDetectionStatus scissor_object_detection_status;
};

constexpr auto kDefaultForce = 0.5;
constexpr auto kDefaultVelocity = 1.0;

// We use this structure to hold our command interface values.
// NOTE: should these be zero initialized or NaN initialized?
struct IndependentControlCommand
{
  double finger_a_position = 0;
  double finger_b_position = 0;
  double finger_c_position = 0;
  double scissor_position = 0;
  double finger_a_velocity = kDefaultVelocity;
  double finger_b_velocity = kDefaultVelocity;
  double finger_c_velocity = kDefaultVelocity;
  double scissor_velocity = kDefaultVelocity;
  double finger_a_force = kDefaultForce;
  double finger_b_force = kDefaultForce;
  double finger_c_force = kDefaultForce;
  double scissor_force = kDefaultForce;
};

/**
 * This is the interface of the driver to control the 3f Gripper.
 * The Driver interface can be easily mocked for testing or implemented to
 * fake the behavior of the real hardware.
 */
class Driver
{
public:
  virtual void set_slave_address(uint8_t slave_address) = 0;

  /** Connect to the gripper serial connection. */
  virtual bool connect() = 0;

  /** Disconnect from the gripper serial connection. */
  virtual void disconnect() = 0;

  /**
   * @brief Activates the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivates the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  virtual void deactivate() = 0;

  virtual FullGripperStatus get_full_status() = 0;

  virtual void send_independent_control_command(IndependentControlCommand const& cmd) = 0;

  virtual void send_simple_control_command(GraspingMode const& mode, double position, double velocity, double force) = 0;

  virtual void clear_faults() = 0;
};

}  // namespace robotiq_3f_driver
