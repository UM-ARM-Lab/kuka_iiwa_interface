#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
// ROS message headers
#include <wiktor_hardware_interface/ControlModeCommand.h>
#include <wiktor_hardware_interface/ControlModeStatus.h>
#include <wiktor_hardware_interface/MotionCommand.h>
#include <wiktor_hardware_interface/MotionStatus.h>
// LCM type headers
#include <wiktor_hardware_interface/control_mode_command.hpp>
#include <wiktor_hardware_interface/control_mode_status.hpp>
#include <wiktor_hardware_interface/motion_command.hpp>
#include <wiktor_hardware_interface/motion_status.hpp>

#ifndef IIWA_HARDWARE_INTERFACE_HPP
#define IIWA_HARDWARE_INTERFACE_HPP

namespace iiwa_hardware_interface
{
    class IIWAHardwareInterface
    {
    public:

        IIWAHardwareInterface();
    };
}

#endif // IIWA_HARDWARE_INTERFACE_HPP
