#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
// ROS message headers
#include <victor_hardware_interface/Robotiq3FingerCommand.h>
#include <victor_hardware_interface/Robotiq3FingerStatus.h>
// LCM type headers
#include <victor_hardware_interface/robotiq_3finger_command.hpp>
#include <victor_hardware_interface/robotiq_3finger_status.hpp>
// LCM
#include <lcm/lcm-cpp.hpp>

#ifndef ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP
#define ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP

namespace robotiq_3finger_hardware_interface
{
    class Robotiq3FingerHardwareInterface
    {
    protected:

        std::shared_ptr<lcm::LCM> send_lcm_ptr_;
        std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
        std::string command_channel_name_;
        std::string status_channel_name_;
        std::function<void(const victor_hardware_interface::Robotiq3FingerStatus&)> status_callback_fn_;

        victor_hardware_interface::robotiq_3finger_actuator_command ConvertFingerCommand(const victor_hardware_interface::Robotiq3FingerActuatorCommand& finger_command) const;

        victor_hardware_interface::Robotiq3FingerActuatorStatus ConvertFingerStatus(const victor_hardware_interface::robotiq_3finger_actuator_status& finger_status) const;

        victor_hardware_interface::Robotiq3FingerObjectStatus ConvertObjectStatus(const victor_hardware_interface::robotiq_3finger_object_status& object_status) const;

        victor_hardware_interface::Robotiq3FingerStatus ConvertStatus(const victor_hardware_interface::robotiq_3finger_status& status) const;

        victor_hardware_interface::robotiq_3finger_command ConvertCommand(const victor_hardware_interface::Robotiq3FingerCommand& command) const;

        void InternalStatusLCMCallback(const lcm::ReceiveBuffer* buffer, const std::string& channel, const victor_hardware_interface::robotiq_3finger_status* status_msg);

    public:

        Robotiq3FingerHardwareInterface(const std::shared_ptr<lcm::LCM>& send_lcm_ptr, const std::shared_ptr<lcm::LCM>& recv_lcm_ptr, const std::string& command_channel_name, const std::string& status_channel_name, const std::function<void(const victor_hardware_interface::Robotiq3FingerStatus&)>& status_callback_fn);

        bool SendCommandMessage(const victor_hardware_interface::Robotiq3FingerCommand& command);
    };
}
#endif // ROBOTIQ_3FINGER_HARDWARE_INTERFACE_HPP
