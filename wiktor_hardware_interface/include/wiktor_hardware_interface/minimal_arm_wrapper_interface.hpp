#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <mutex>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/maybe.hpp>
#include <wiktor_hardware_interface/iiwa_hardware_interface.hpp>
#include <wiktor_hardware_interface/robotiq_3finger_hardware_interface.hpp>
// ROS message headers
#include <wiktor_hardware_interface/ControlModeCommand.h>
#include <wiktor_hardware_interface/ControlModeStatus.h>
#include <wiktor_hardware_interface/MotionCommand.h>
#include <wiktor_hardware_interface/MotionStatus.h>
#include <wiktor_hardware_interface/Robotiq3FingerCommand.h>
#include <wiktor_hardware_interface/Robotiq3FingerStatus.h>
#include <wiktor_hardware_interface/SetControlMode.h>
#include <wiktor_hardware_interface/GetControlMode.h>
// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
// LCM
#include <lcm/lcm-cpp.hpp>

#ifndef MINIMAL_ARM_WRAPPER_INTERFACE_HPP
#define MINIMAL_ARM_WRAPPER_INTERFACE_HPP

namespace wiktor_hardware_interface
{
    class MinimalArmWrapperInterface
    {
    protected:

        ros::NodeHandle nh_;
        ros::Publisher motion_status_pub_;
        ros::Publisher control_mode_status_pub_;
        ros::Publisher gripper_status_pub_;
        ros::Subscriber motion_command_sub_;
        ros::Subscriber gripper_command_sub_;
        ros::ServiceServer set_control_mode_server_;
        ros::ServiceServer get_control_mode_server_;
        ros::CallbackQueue ros_callback_queue_;
        std::thread ros_callback_thread_;

        Maybe::Maybe<wiktor_hardware_interface::ControlModeStatus> active_control_mode_;
        std::mutex control_mode_status_mutex_;
        const double set_control_mode_timeout_;

        std::shared_ptr<lcm::LCM> send_lcm_ptr_;
        std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
        std::unique_ptr<iiwa_hardware_interface::IIWAHardwareInterface> iiwa_ptr_;
        std::unique_ptr<robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface> robotiq_ptr_;

    public:

        MinimalArmWrapperInterface(ros::NodeHandle& nh, const std::string& motion_command_topic, const std::string& motion_status_topic, const std::string& control_mode_status_topic, const std::string& get_control_mode_service, const std::string& set_control_mode_service, const std::string& gripper_command_topic, const std::string& gripper_status_topic, const std::shared_ptr<lcm::LCM>& send_lcm_ptr, const std::shared_ptr<lcm::LCM>& recv_lcm_ptr, const std::string& motion_command_channel, const std::string& motion_status_channel, const std::string& control_mode_command_channel, const std::string& control_mode_status_channel, const std::string& gripper_command_channel, const std::string& gripper_status_channel, const double set_control_mode_timeout);

        void ROSCallbackThread();

        void LCMLoop();

        static inline bool JVQMatch(const wiktor_hardware_interface::JointValueQuantity& jvq1, const wiktor_hardware_interface::JointValueQuantity& jvq2)
        {
            if (jvq1.joint_1 != jvq2.joint_1)
            {
                return false;
            }
            else if (jvq1.joint_2 != jvq2.joint_2)
            {
                return false;
            }
            else if (jvq1.joint_3 != jvq2.joint_3)
            {
                return false;
            }
            else if (jvq1.joint_4 != jvq2.joint_4)
            {
                return false;
            }
            else if (jvq1.joint_5 != jvq2.joint_5)
            {
                return false;
            }
            else if (jvq1.joint_6 != jvq2.joint_6)
            {
                return false;
            }
            else if (jvq1.joint_7 != jvq2.joint_7)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        static inline bool CVQMatch(const wiktor_hardware_interface::CartesianValueQuantity& cvq1, const wiktor_hardware_interface::CartesianValueQuantity& cvq2)
        {
            if (cvq1.x != cvq2.x)
            {
                return false;
            }
            else if (cvq1.y != cvq2.y)
            {
                return false;
            }
            else if (cvq1.z != cvq2.z)
            {
                return false;
            }
            else if (cvq1.a != cvq2.a)
            {
                return false;
            }
            else if (cvq1.b != cvq2.b)
            {
                return false;
            }
            else if (cvq1.c != cvq2.c)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        static inline bool PExPMatch(const wiktor_hardware_interface::PathExecutionParameters& pexp1, const wiktor_hardware_interface::PathExecutionParameters& pexp2)
        {
            if (pexp1.joint_relative_acceleration != pexp2.joint_relative_acceleration)
            {
                return false;
            }
            else if (pexp1.joint_relative_velocity != pexp2.joint_relative_velocity)
            {
                return false;
            }
            else if (pexp1.override_joint_acceleration != pexp2.override_joint_acceleration)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        bool CheckControlModeCommandAndStatusMatch(const wiktor_hardware_interface::ControlModeCommand& command, const wiktor_hardware_interface::ControlModeStatus& status) const;

        std::pair<bool, std::string> SafetyCheckPathExecutionParams(const wiktor_hardware_interface::PathExecutionParameters& params) const;

        std::pair<bool, std::string> SafetyCheckJointImpedanceParameters(const wiktor_hardware_interface::JointImpedanceParameters& params) const;

        std::pair<bool, std::string> SafetyCheckCartesianImpedanceParameters(const wiktor_hardware_interface::CartesianImpedanceParameters& params) const;

        std::pair<bool, std::string> SafetyCheckCartesianControlModeLimits(const wiktor_hardware_interface::CartesianControlModeLimits& params) const;

        std::pair<bool, std::string> SafetyCheckControlMode(const wiktor_hardware_interface::ControlModeCommand& control_mode) const;

        wiktor_hardware_interface::ControlModeCommand MergeControlModeCommand(const wiktor_hardware_interface::ControlModeStatus& active_control_mode, const wiktor_hardware_interface::ControlModeCommand& new_control_mode) const;

        bool SetControlModeCallback(wiktor_hardware_interface::SetControlMode::Request& req, wiktor_hardware_interface::SetControlMode::Response& res);

        bool GetControlModeCallback(wiktor_hardware_interface::GetControlMode::Request& req, wiktor_hardware_interface::GetControlMode::Response& res);

        bool SafetyCheckPositions(const wiktor_hardware_interface::JointValueQuantity& positions) const;

        bool SafetyCheckPositionsVelocities(const wiktor_hardware_interface::JointValueQuantity& positions, const wiktor_hardware_interface::JointValueQuantity& velocities) const;

        bool SafetyCheckCartesianPose(const wiktor_hardware_interface::CartesianValueQuantity& pose) const;

        bool SafetyCheckMotionCommand(const wiktor_hardware_interface::MotionCommand& command);

        void MotionCommandROSCallback(wiktor_hardware_interface::MotionCommand command);

        bool SafetyCheckFingerCommand(const wiktor_hardware_interface::Robotiq3FingerActuatorCommand& command) const;

        bool SafetyCheckGripperCommand(const wiktor_hardware_interface::Robotiq3FingerCommand& command) const;

        void GripperCommandROSCallback(wiktor_hardware_interface::Robotiq3FingerCommand command);

        void MotionStatusLCMCallback(const wiktor_hardware_interface::MotionStatus& motion_status);

        void ControlModeStatusLCMCallback(const wiktor_hardware_interface::ControlModeStatus& control_mode_status);

        void GripperStatusLCMCallback(const wiktor_hardware_interface::Robotiq3FingerStatus& gripper_status);
    };
}

#endif // MINIMAL_ARM_WRAPPER_INTERFACE_HPP
