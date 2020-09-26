#include <string>
#include <lcm/lcm-cpp.hpp>
// ROS message headers
#include "victor_hardware_interface_msgs/ControlModeParameters.h"
#include "victor_hardware_interface_msgs/MotionCommand.h"
#include "victor_hardware_interface_msgs/MotionStatus.h"
// LCM type headers
#include "victor_hardware_interface/control_mode_parameters.hpp"
#include "victor_hardware_interface/motion_command.hpp"
#include "victor_hardware_interface/motion_status.hpp"

#ifndef IIWA_HARDWARE_INTERFACE_HPP
#define IIWA_HARDWARE_INTERFACE_HPP

namespace victor_hardware_interface
{
    namespace msgs = victor_hardware_interface_msgs;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LCM to ROS and ROS to LCM message converters
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    msgs::JointValueQuantity jvqLcmToRos(const joint_value_quantity& lcm_jvq);
    joint_value_quantity jvqRosToLcm( const msgs::JointValueQuantity& ros_jvq);

    msgs::CartesianValueQuantity cvqLcmToRos(const cartesian_value_quantity& lcm_cvq);
    cartesian_value_quantity cvqRosToLcm(const msgs::CartesianValueQuantity& ros_cvq);

    geometry_msgs::Pose poseLcmToRos(const cartesian_pose& lcm_pose);
    cartesian_pose poseRosToLcm(const geometry_msgs::Pose& ros_pose);

    msgs::JointImpedanceParameters jointImpedanceParamsLcmToRos(const joint_impedance_parameters& lcm_jip);
    joint_impedance_parameters jointImpedanceParamsRosToLcm(const msgs::JointImpedanceParameters& ros_jip);

    msgs::CartesianImpedanceParameters cartesianImpedanceParamsLcmToRos(const cartesian_impedance_parameters& lcm_cip);
    cartesian_impedance_parameters cartesianImpedanceParamsRosToLcm(const msgs::CartesianImpedanceParameters& ros_cip);

    msgs::JointPathExecutionParameters jointPexpLcmToRos(const joint_path_execution_parameters& path_execution_params);
    joint_path_execution_parameters jointPexpRosToLcm(const msgs::JointPathExecutionParameters& path_execution_params);

    msgs::CartesianPathExecutionParameters cartesianPexpLcmToRos(const cartesian_path_execution_parameters& lcm_pexp);
    cartesian_path_execution_parameters cartesianPexpRosToLcm(const msgs::CartesianPathExecutionParameters& ros_pexp);

    msgs::CartesianControlModeLimits cartesianControlModeLimitsLcmToRos(const cartesian_control_mode_limits& lcm_ccml);
    cartesian_control_mode_limits cartesianControlModeLimitsRosToLcm(const msgs::CartesianControlModeLimits& ros_ccml);

    msgs::ControlMode controlModeLcmToRos(const control_mode& lcm_cm);
    control_mode controlModeRosToLcm(const msgs::ControlMode& ros_cm);

    msgs::MotionStatus motionStatusLcmToRos(const motion_status& lcm_status);
    motion_command motionCommandRosToLcm(const msgs::MotionCommand& ros_command);

    msgs::ControlModeParameters controlModeParamsLcmToRos(const control_mode_parameters& lcm_cmp);
    control_mode_parameters controlModeParamsRosToLcm(const msgs::ControlModeParameters& ros_cmp);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The class that does the actual communication
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class IIWAHardwareInterface
    {
    protected:

        std::shared_ptr<lcm::LCM> send_lcm_ptr_;
        std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
        std::string motion_command_channel_name_;
        std::string motion_status_channel_name_;
        std::function<void(const msgs::MotionStatus&)> motion_status_callback_fn_;
        std::string control_mode_command_channel_name_;
        std::string control_mode_status_channel_name_;
        std::function<void(const msgs::ControlModeParameters&)> control_mode_status_callback_fn_;

        void InternalMotionStatusLCMCallback(
                const lcm::ReceiveBuffer* buffer,
                const std::string& channel,
                const motion_status* status_msg);

        void InternalControlModeStatusLCMCallback(
                const lcm::ReceiveBuffer* buffer,
                const std::string& channel,
                const control_mode_parameters* status_msg);

    public:

        IIWAHardwareInterface(
                const std::shared_ptr<lcm::LCM>& send_lcm_ptr,
                const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
                const std::string& motion_command_channel_name,
                const std::string& motion_status_channel_name,
                const std::function<void(const msgs::MotionStatus&)>& motion_status_callback_fn,
                const std::string& control_mode_command_channel_name,
                const std::string& control_mode_status_channel_name,
                const std::function<void(const msgs::ControlModeParameters&)>& control_mode_status_callback_fn);

        bool SendMotionCommandMessage(const msgs::MotionCommand& command);

        bool SendControlModeCommandMessage(const msgs::ControlModeParameters& command);
    };
}

#endif // IIWA_HARDWARE_INTERFACE_HPP
