#include "victor_hardware_interface/minimal_arm_wrapper_interface.hpp"

// ROS
#include <ros/callback_queue.h>

// ROS message headers
#include "victor_hardware_interface/ControlModeCommand.h"
#include "victor_hardware_interface/ControlModeStatus.h"
#include "victor_hardware_interface/MotionCommand.h"
#include "victor_hardware_interface/MotionStatus.h"
#include "victor_hardware_interface/Robotiq3FingerCommand.h"
#include "victor_hardware_interface/Robotiq3FingerStatus.h"
#include "victor_hardware_interface/SetControlMode.h"
#include "victor_hardware_interface/GetControlMode.h"

// LCM
#include "victor_hardware_interface/iiwa_hardware_interface.hpp"
#include "victor_hardware_interface/robotiq_3finger_hardware_interface.hpp"

namespace victor_hardware_interface
{
    inline victor_hardware_interface::ControlModeCommand mergeControlModeCommand(const victor_hardware_interface::ControlModeStatus& active_control_mode, 
                                                                                 const victor_hardware_interface::ControlModeCommand& new_control_mode);

    MinimalArmWrapperInterface::MinimalArmWrapperInterface(
            ros::NodeHandle& nh,
            const std::shared_ptr<lcm::LCM>& send_lcm_ptr,
            const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
            const std::string& cartesian_control_frame,
            const double set_control_mode_timeout,
            // ROS Topics
            const std::string& motion_command_topic,
            const std::string& motion_status_topic,
            const std::string& control_mode_status_topic,
            const std::string& get_control_mode_service,
            const std::string& set_control_mode_service,
            const std::string& gripper_command_topic,
            const std::string& gripper_status_topic,
            // LCM channels
            const std::string& motion_command_channel,
            const std::string& motion_status_channel,
            const std::string& control_mode_command_channel,
            const std::string& control_mode_status_channel,
            const std::string& gripper_command_channel,
            const std::string& gripper_status_channel)
        : nh_(nh)
        , cartesian_control_frame_(cartesian_control_frame)
        , set_control_mode_timeout_(set_control_mode_timeout)
        , send_lcm_ptr_(send_lcm_ptr)
        , recv_lcm_ptr_(recv_lcm_ptr)
    {
        if (cartesian_control_frame_ == "")
        {
            throw std::invalid_argument("Cartesian control frame [""] is not valid");
        }
        nh_.setCallbackQueue(&ros_callback_queue_);

        // Set up IIWA LCM interface
        const auto motion_status_callback_fn = [&] (const victor_hardware_interface::MotionStatus& motion_status)
        {
            return motionStatusLCMCallback(motion_status);
        };
        const auto control_mode_status_callback_fn = [&] (const victor_hardware_interface::ControlModeStatus& control_mode_status)
        {
            return controlModeStatusLCMCallback(control_mode_status);
        };
        iiwa_ptr_ = std::unique_ptr<iiwa_hardware_interface::IIWAHardwareInterface>(
                    new iiwa_hardware_interface::IIWAHardwareInterface(
                        send_lcm_ptr_, recv_lcm_ptr_,
                        motion_command_channel, motion_status_channel, motion_status_callback_fn,
                        control_mode_command_channel, control_mode_status_channel, control_mode_status_callback_fn));

        // Set up Robotiq LCM interface
        const auto gripper_status_callback_fn = [&] (const victor_hardware_interface::Robotiq3FingerStatus& gripper_status)
        {
            return gripperStatusLCMCallback(gripper_status);
        };
        robotiq_ptr_ = std::unique_ptr<robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface>(
                    new robotiq_3finger_hardware_interface::Robotiq3FingerHardwareInterface(
                        send_lcm_ptr_, recv_lcm_ptr_,
                        gripper_command_channel, gripper_status_channel, gripper_status_callback_fn));

        // Set up ROS interfaces
        motion_status_pub_ = nh_.advertise<victor_hardware_interface::MotionStatus>(motion_status_topic, 1, false);
        control_mode_status_pub_ = nh_.advertise<victor_hardware_interface::ControlModeStatus>(control_mode_status_topic, 1, false);
        gripper_status_pub_ = nh_.advertise<victor_hardware_interface::Robotiq3FingerStatus>(gripper_status_topic, 1, false);
        motion_command_sub_ = nh_.subscribe(motion_command_topic, 1, &MinimalArmWrapperInterface::motionCommandROSCallback, this);
        gripper_command_sub_ = nh_.subscribe(gripper_command_topic, 1, &MinimalArmWrapperInterface::gripperCommandROSCallback, this);;
        set_control_mode_server_ = nh_.advertiseService(set_control_mode_service, &MinimalArmWrapperInterface::setControlModeCallback, this);
        get_control_mode_server_ = nh_.advertiseService(get_control_mode_service, &MinimalArmWrapperInterface::getControlModeCallback, this);

        // Start ROS thread
        ros_callback_thread_ = std::thread(std::bind(&MinimalArmWrapperInterface::ROSCallbackThread, this));
    }

    void MinimalArmWrapperInterface::ROSCallbackThread()
    {
        const double timeout = 0.001;
        while (nh_.ok())
        {
            ros_callback_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    void MinimalArmWrapperInterface::LCMLoop()
    {
        ROS_INFO_STREAM_NAMED(ros::this_node::getName(), "Starting LCM spin loop...");
        // Run LCM
        bool lcm_ok = true;
        while (ros::ok() && lcm_ok)
        {
            bool lcm_running = true;
            while (lcm_running)
            {
                const int ret = recv_lcm_ptr_->handleTimeout(1);
                if (ret > 0)
                {
                    lcm_running = true;
                }
                else if (ret == 0)
                {
                    lcm_running = false;
                }
                else
                {
                    lcm_running = false;
                    lcm_ok = false;
                    ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "LCM error " << ret);
                }
            }
        }
    }

    bool MinimalArmWrapperInterface::JVQMatch(const victor_hardware_interface::JointValueQuantity& jvq1, const victor_hardware_interface::JointValueQuantity& jvq2)
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

    bool MinimalArmWrapperInterface::CVQMatch(const victor_hardware_interface::CartesianValueQuantity& cvq1, const victor_hardware_interface::CartesianValueQuantity& cvq2)
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

    bool MinimalArmWrapperInterface::jointPExPMatch(const victor_hardware_interface::JointPathExecutionParameters& pexp1, const victor_hardware_interface::JointPathExecutionParameters& pexp2)
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

    bool MinimalArmWrapperInterface::cartesianPExPMatch(const victor_hardware_interface::CartesianPathExecutionParameters& pexp1, const victor_hardware_interface::CartesianPathExecutionParameters& pexp2)
    {
        if (CVQMatch(pexp1.max_velocity, pexp2.max_velocity) == false)
        {
            return false;
        }
        else if (CVQMatch(pexp1.max_acceleration, pexp2.max_acceleration) == false)
        {
            return false;
        }
        else if (pexp1.max_nullspace_velocity != pexp2.max_nullspace_velocity)
        {
            return false;
        }
        else if (pexp1.max_nullspace_acceleration != pexp2.max_nullspace_acceleration)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool MinimalArmWrapperInterface::checkControlModeCommandAndStatusMatch(const victor_hardware_interface::ControlModeCommand& command, const victor_hardware_interface::ControlModeStatus& status)
    {
        //std::cout << "ControlModeCommand:\n" << command << "\nControlModeStatus:\n" << status << std::endl;
        const bool cdmatch = CVQMatch(command.cartesian_impedance_params.cartesian_damping, status.cartesian_impedance_params.cartesian_damping);
        const bool ndmatch = (command.cartesian_impedance_params.nullspace_damping == status.cartesian_impedance_params.nullspace_damping);
        const bool csmatch = CVQMatch(command.cartesian_impedance_params.cartesian_stiffness, status.cartesian_impedance_params.cartesian_stiffness);
        const bool nsmatch = (command.cartesian_impedance_params.nullspace_stiffness == status.cartesian_impedance_params.nullspace_stiffness);
        const bool mpd_match = CVQMatch(command.cartesian_control_mode_limits.max_path_deviation, status.cartesian_control_mode_limits.max_path_deviation);
        const bool mcv_match = CVQMatch(command.cartesian_control_mode_limits.max_cartesian_velocity, status.cartesian_control_mode_limits.max_cartesian_velocity);
        const bool mcf_match = CVQMatch(command.cartesian_control_mode_limits.max_control_force, status.cartesian_control_mode_limits.max_control_force);
        const bool smcf_match = (command.cartesian_control_mode_limits.stop_on_max_control_force == status.cartesian_control_mode_limits.stop_on_max_control_force);
        const bool jdmatch = JVQMatch(command.joint_impedance_params.joint_damping, status.joint_impedance_params.joint_damping);
        const bool jsmatch = JVQMatch(command.joint_impedance_params.joint_stiffness, status.joint_impedance_params.joint_stiffness);
        const bool jpexpmatch = jointPExPMatch(command.joint_path_execution_params, status.joint_path_execution_params);
        const bool cpexpmatch = cartesianPExPMatch(command.cartesian_path_execution_params, status.cartesian_path_execution_params);
        const bool cmmatch = (command.control_mode == status.active_control_mode);
        if (cdmatch && ndmatch && csmatch && nsmatch && mpd_match && mcv_match && mcf_match && smcf_match && jdmatch && jsmatch && jpexpmatch && cpexpmatch && cmmatch)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::safetyCheckJointPathExecutionParams(const victor_hardware_interface::JointPathExecutionParameters& params) const
    {
        bool valid = true;
        std::string message;
        if (params.joint_relative_velocity <= 0.0 || params.joint_relative_velocity > 1.0)
        {
            valid = false;
            message += "+Invalid joint relative velocity";
        }
        if (params.joint_relative_acceleration <= 0.0 || params.joint_relative_acceleration > 1.0)
        {
            valid = false;
            message += "+Invalid joint relative acceleration";
        }
        if (params.override_joint_acceleration < 0.0 || params.override_joint_acceleration > 10.0)
        {
            valid = false;
            message += "+Invalid override joint acceleration";
        }
        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::safetyCheckCartesianPathExecutionParams(const victor_hardware_interface::CartesianPathExecutionParameters& params) const
    {
        bool valid = true;
        std::string message;
        if (params.max_velocity.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max velocity";
        }
        if (params.max_velocity.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max velocity";
        }
        if (params.max_velocity.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max velocity";
        }
        if (params.max_velocity.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max velocity";
        }
        if (params.max_velocity.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max velocity";
        }
        if (params.max_velocity.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max velocity";
        }
        if (params.max_nullspace_velocity <= 0.0)
        {
            valid = false;
            message += "+Invalid nullspace max velocity";
        }
        if (params.max_acceleration.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max acceleration";
        }
        if (params.max_acceleration.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max acceleration";
        }
        if (params.max_acceleration.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max acceleration";
        }
        if (params.max_acceleration.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max acceleration";
        }
        if (params.max_acceleration.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max acceleration";
        }
        if (params.max_acceleration.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max acceleration";
        }
        if (params.max_nullspace_acceleration <= 0.0)
        {
            valid = false;
            message += "+Invalid nullspace max acceleration";
        }
        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::safetyCheckJointImpedanceParams(const victor_hardware_interface::JointImpedanceParameters& params) const
    {
        bool valid = true;
        std::string message;
        if (params.joint_damping.joint_1 < 0.0 || params.joint_damping.joint_1 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 1 damping";
        }
        if (params.joint_damping.joint_2 < 0.0 || params.joint_damping.joint_2 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 2 damping";
        }
        if (params.joint_damping.joint_3 < 0.0 || params.joint_damping.joint_3 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 3 damping";
        }
        if (params.joint_damping.joint_4 < 0.0 || params.joint_damping.joint_4 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 4 damping";
        }
        if (params.joint_damping.joint_5 < 0.0 || params.joint_damping.joint_5 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 5 damping";
        }
        if (params.joint_damping.joint_6 < 0.0 || params.joint_damping.joint_6 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 6 damping";
        }
        if (params.joint_damping.joint_7 < 0.0 || params.joint_damping.joint_7 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 7 damping";
        }
        if (params.joint_stiffness.joint_1 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 1 stiffness";
        }
        if (params.joint_stiffness.joint_2 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 2 stiffness";
        }
        if (params.joint_stiffness.joint_3 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 3 stiffness";
        }
        if (params.joint_stiffness.joint_4 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 4 stiffness";
        }
        if (params.joint_stiffness.joint_5 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 5 stiffness";
        }
        if (params.joint_stiffness.joint_6 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 6 stiffness";
        }
        if (params.joint_stiffness.joint_7 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 7 stiffness";
        }
        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::safetyCheckCartesianImpedanceParams(const victor_hardware_interface::CartesianImpedanceParameters& params) const
    {
        bool valid = true;
        std::string message;
        if (params.cartesian_damping.x < 0.1 || params.cartesian_damping.x > 1.0)
        {
            valid = false;
            message += "+Invalid DoF X damping";
        }
        if (params.cartesian_damping.y < 0.1 || params.cartesian_damping.y > 1.0)
        {
            valid = false;
            message += "+Invalid DoF Y damping";
        }
        if (params.cartesian_damping.z < 0.1 || params.cartesian_damping.z > 1.0)
        {
            valid = false;
            message += "+Invalid DoF Z damping";
        }
        if (params.cartesian_damping.a < 0.1 || params.cartesian_damping.a > 1.0)
        {
            valid = false;
            message += "+Invalid DoF A damping";
        }
        if (params.cartesian_damping.b < 0.1 || params.cartesian_damping.b > 1.0)
        {
            valid = false;
            message += "+Invalid DoF B damping";
        }
        if (params.cartesian_damping.c < 0.1 || params.cartesian_damping.c > 1.0)
        {
            valid = false;
            message += "+Invalid DoF C damping";
        }
        if (params.nullspace_damping < 0.3 || params.nullspace_damping > 1.0)
        {
            valid = false;
            message += "+Invalid nullspace damping";
        }
        if (params.cartesian_stiffness.x < 0.0 || params.cartesian_stiffness.x > 5000.0)
        {
            valid = false;
            message += "+Invalid DoF X stiffness";
        }
        if (params.cartesian_stiffness.y < 0.0 || params.cartesian_stiffness.y > 5000.0)
        {
            valid = false;
            message += "+Invalid DoF Y stiffness";
        }
        if (params.cartesian_stiffness.z < 0.0 || params.cartesian_stiffness.z > 5000.0)
        {
            valid = false;
            message += "+Invalid DoF Z stiffness";
        }
        if (params.cartesian_stiffness.a < 0.1 || params.cartesian_stiffness.a > 300.0)
        {
            valid = false;
            message += "+Invalid DoF A stiffness";
        }
        if (params.cartesian_stiffness.b < 0.1 || params.cartesian_stiffness.b > 300.0)
        {
            valid = false;
            message += "+Invalid DoF B stiffness";
        }
        if (params.cartesian_stiffness.c < 0.1 || params.cartesian_stiffness.c > 300.0)
        {
            valid = false;
            message += "+Invalid DoF C stiffness";
        }
        if (params.nullspace_stiffness < 0.0)
        {
            valid = false;
            message += "+Invalid nullspace stiffness";
        }
        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::safetyCheckCartesianControlModeLimits(const victor_hardware_interface::CartesianControlModeLimits& params) const
    {
        bool valid = true;
        std::string message;
        if (params.max_path_deviation.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max path deviation";
        }
        if (params.max_path_deviation.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max path deviation";
        }
        if (params.max_path_deviation.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max path deviation";
        }
        if (params.max_path_deviation.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max path deviation";
        }
        if (params.max_path_deviation.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max path deviation";
        }
        if (params.max_path_deviation.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max path deviation";
        }
        if (params.max_cartesian_velocity.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max cartesian velocity";
        }
        if (params.max_cartesian_velocity.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max cartesian velocity";
        }
        if (params.max_cartesian_velocity.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max cartesian velocity";
        }
        if (params.max_cartesian_velocity.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max cartesian velocity";
        }
        if (params.max_cartesian_velocity.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max cartesian velocity";
        }
        if (params.max_cartesian_velocity.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max cartesian velocity";
        }
        if (params.max_control_force.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max control force";
        }
        if (params.max_control_force.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max control force";
        }
        if (params.max_control_force.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max control force";
        }
        if (params.max_control_force.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max control force";
        }
        if (params.max_control_force.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max control force";
        }
        if (params.max_control_force.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max control force";
        }
        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::safetyCheckControlMode(const victor_hardware_interface::ControlModeCommand& control_mode) const
    {
        bool valid = true;
        std::string message;
        if (control_mode.control_mode != victor_hardware_interface::ControlModeCommand::JOINT_POSITION
                && control_mode.control_mode != victor_hardware_interface::ControlModeCommand::JOINT_IMPEDANCE
                && control_mode.control_mode != victor_hardware_interface::ControlModeCommand::CARTESIAN_POSE
                && control_mode.control_mode != victor_hardware_interface::ControlModeCommand::CARTESIAN_IMPEDANCE)
        {
            valid = false;
            message += "+Invalid control mode";
        }
        const auto valid_joint_impedance_params = safetyCheckJointImpedanceParams(control_mode.joint_impedance_params);
        message += valid_joint_impedance_params.second;
        if (!valid_joint_impedance_params.first)
        {
            valid = false;
        }
        const auto valid_cartesian_impedance_params = safetyCheckCartesianImpedanceParams(control_mode.cartesian_impedance_params);
        message += valid_cartesian_impedance_params.second;
        if (!valid_cartesian_impedance_params.first)
        {
            valid = false;
        }
        const auto valid_cartesian_control_mode_limits = safetyCheckCartesianControlModeLimits(control_mode.cartesian_control_mode_limits);
        message += valid_cartesian_control_mode_limits.second;
        if (!valid_cartesian_control_mode_limits.first)
        {
            valid = false;
        }
        const auto joint_valid_path_execution_params = safetyCheckJointPathExecutionParams(control_mode.joint_path_execution_params);
        message += joint_valid_path_execution_params.second;
        if (!joint_valid_path_execution_params.first)
        {
            valid = false;
        }
        const auto valid_cartesian_path_execution_params = safetyCheckCartesianPathExecutionParams(control_mode.cartesian_path_execution_params);
        message += valid_cartesian_path_execution_params.second;
        if (!valid_cartesian_path_execution_params.first)
        {
            valid = false;
        }
        return std::make_pair(valid, message);
    }

    bool MinimalArmWrapperInterface::setControlModeCallback(victor_hardware_interface::SetControlMode::Request& req, victor_hardware_interface::SetControlMode::Response& res)
    {
        Maybe::Maybe<victor_hardware_interface::ControlModeStatus> local_active_control_mode_copy;
        {
            std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
            local_active_control_mode_copy = active_control_mode_;
        }
        if (local_active_control_mode_copy.Valid())
        {
            const victor_hardware_interface::ControlModeCommand merged_command = mergeControlModeCommand(local_active_control_mode_copy.GetImmutable(), req.new_control_mode);
            //std::cout << "++++++++++++++++++++++++++++++++++++++++\nMerged control mode:\n" << merged_command << std::endl;
            const std::pair<bool, std::string> safety_check = safetyCheckControlMode(merged_command);
            if (safety_check.first)
            {
                iiwa_ptr_->SendControlModeCommandMessage(merged_command);
                // Loop waiting for a matching control mode to be parsed
                bool control_mode_matches = false;

                const std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
                std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
                
                do
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    end_time = std::chrono::high_resolution_clock::now();
                    std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
                    control_mode_matches = checkControlModeCommandAndStatusMatch(merged_command, active_control_mode_.GetImmutable());
                }
                while (!control_mode_matches && std::chrono::duration<double>(end_time - start_time).count() < set_control_mode_timeout_);

                // Check the results of the timeout
                if (control_mode_matches)
                {
                    res.success = true;
                    res.message = "Control mode set successfully";
                }
                else
                {
                    res.success = false;
                    res.message = "Control mode could not be set in Sunrise";
                }
            }
            else
            {
                res.success = false;
                res.message = safety_check.second;
            }
        }
        else
        {
            res.success = false;
            res.message = "No initial control mode available from the controller";
        }
        return true;
    }

    bool MinimalArmWrapperInterface::getControlModeCallback(victor_hardware_interface::GetControlMode::Request& req, victor_hardware_interface::GetControlMode::Response& res)
    {
        UNUSED(req);
        std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
        res.has_active_control_mode = active_control_mode_.Valid();
        if (res.has_active_control_mode)
        {
            res.active_control_mode = active_control_mode_.Get();
        }
        return true;
    }

    bool MinimalArmWrapperInterface::safetyCheckPositions(const victor_hardware_interface::JointValueQuantity& positions) const
    {
        //TODO: The function is not implemented yet, and its name may be changed to CheckJointPositionLimits since it has nothing to do with safety.
        UNUSED(positions);
        return true;
    }

    bool MinimalArmWrapperInterface::safetyCheckPositionsVelocities(const victor_hardware_interface::JointValueQuantity& positions, const victor_hardware_interface::JointValueQuantity& velocities) const
    {
        //TODO: The function is not implemented yet. 
        UNUSED(positions);
        UNUSED(velocities);
        return true;
    }

    bool MinimalArmWrapperInterface::safetyCheckCartesianPose(const geometry_msgs::Pose& pose, const std::string& frame) const
    {
        // Check to make sure the frame is correct!
        if (frame != cartesian_control_frame_)
        {
            ROS_ERROR_NAMED(ros::this_node::getName(), "Commanded cartesian pose has the wrong frame, %s given, %s expected", frame.c_str(), cartesian_control_frame_.c_str());
            return false;
        }
        else
        {
            const double quat_squared_norm = (pose.orientation.w * pose.orientation.w)
                                             + (pose.orientation.x * pose.orientation.x)
                                             + (pose.orientation.y * pose.orientation.y)
                                             + (pose.orientation.z * pose.orientation.z);
            const double error = std::fabs(1.0 - quat_squared_norm);
            if (error > 1e-6)
            {
                ROS_ERROR_NAMED(ros::this_node::getName(), "Commanded cartesian pose quaternion is not normalized, squared norm = %f", quat_squared_norm);
                return false;
            }
        }
        return true;
    }

    bool MinimalArmWrapperInterface::safetyCheckMotionCommand(const victor_hardware_interface::MotionCommand& command)
    {
        std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
        if (active_control_mode_.Valid())
        {
            const uint8_t active_control_type = active_control_mode_.GetImmutable().active_control_mode;
            const uint8_t command_motion_type = command.control_mode;
            if (active_control_type == victor_hardware_interface::ControlModeCommand::JOINT_POSITION)
            {
                if (command_motion_type == victor_hardware_interface::MotionCommand::JOINT_POSITION)
                {
                    return safetyCheckPositions(command.joint_position);
                }
                else
                {
                    return false;
                }
            }
            else if (active_control_type == victor_hardware_interface::ControlModeCommand::JOINT_IMPEDANCE)
            {
                if (command_motion_type == victor_hardware_interface::MotionCommand::JOINT_POSITION)
                {
                    return safetyCheckPositions(command.joint_position);
                }
                else
                {
                    return false;
                }
            }
            else if (active_control_type == victor_hardware_interface::ControlModeCommand::CARTESIAN_POSE)
            {
                if (command_motion_type == victor_hardware_interface::MotionCommand::CARTESIAN_POSE)
                {
                    return safetyCheckCartesianPose(command.cartesian_pose, command.header.frame_id);
                }
                else
                {
                    return false;
                }
            }
            else if (active_control_type == victor_hardware_interface::ControlModeCommand::CARTESIAN_IMPEDANCE)
            {
                if (command_motion_type == victor_hardware_interface::MotionCommand::CARTESIAN_IMPEDANCE)
                {
                    return safetyCheckCartesianPose(command.cartesian_pose, command.header.frame_id);
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    void MinimalArmWrapperInterface::motionCommandROSCallback(victor_hardware_interface::MotionCommand command)
    {
        if (safetyCheckMotionCommand(command))
        {
            iiwa_ptr_->SendMotionCommandMessage(command);
        }
        else
        {
            ROS_WARN_NAMED(ros::this_node::getName(), "Motion command failed safety checks");
        }
    }

    bool MinimalArmWrapperInterface::safetyCheckFingerCommand(const victor_hardware_interface::Robotiq3FingerActuatorCommand& command) const
    {
        if (command.position > 1.0 || command.position < 0.0)
        {
            return false;
        }
        
        if (command.force > 1.0 || command.force < 0.0)
        {
            return false;
        }
        
        if (command.speed > 1.0 || command.speed < 0.0)
        {
            return false;
        }
        
        return true;
    }

    bool MinimalArmWrapperInterface::safetyCheckGripperCommand(const victor_hardware_interface::Robotiq3FingerCommand& command) const
    {
        const bool ac = safetyCheckFingerCommand(command.finger_a_command);
        const bool bc = safetyCheckFingerCommand(command.finger_b_command);
        const bool cc = safetyCheckFingerCommand(command.finger_c_command);
        const bool sc = safetyCheckFingerCommand(command.scissor_command);
        if (ac && bc && cc && sc)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void MinimalArmWrapperInterface::gripperCommandROSCallback(victor_hardware_interface::Robotiq3FingerCommand command)
    {
        if (safetyCheckGripperCommand(command))
        {
            robotiq_ptr_->SendCommandMessage(command);
        }
    }

    void MinimalArmWrapperInterface::motionStatusLCMCallback(const victor_hardware_interface::MotionStatus& motion_status)
    {
        motion_status_pub_.publish(motion_status);
    }

    void MinimalArmWrapperInterface::controlModeStatusLCMCallback(const victor_hardware_interface::ControlModeStatus& control_mode_status)
    {
        {
            std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
            if (active_control_mode_.Valid() == false)
            {
                ROS_INFO_STREAM_NAMED(ros::this_node::getName(),"Initializing active_control_mode for the first time");
            }
            active_control_mode_ = control_mode_status;
        }
        control_mode_status_pub_.publish(control_mode_status);
    }

    void MinimalArmWrapperInterface::gripperStatusLCMCallback(const victor_hardware_interface::Robotiq3FingerStatus& gripper_status)
    {
        gripper_status_pub_.publish(gripper_status);
    }

    inline bool jointPathExecutionParamsIsDefault(const victor_hardware_interface::JointPathExecutionParameters& params)
    {
        return (params.joint_relative_velocity == 0 && params.joint_relative_acceleration == 0 && params.override_joint_acceleration == 0);
    }

    inline bool cartesianPathExecutionParamsIsDefault(const victor_hardware_interface::CartesianPathExecutionParameters& params)
    {
        return (params.max_velocity.x == 0 && params.max_velocity.y == 0 && params.max_velocity.z == 0 && 
                params.max_velocity.a == 0 && params.max_velocity.b == 0 && params.max_velocity.c == 0 &&
                params.max_acceleration.x == 0 && params.max_acceleration.y == 0 && params.max_acceleration.z == 0 && 
                params.max_acceleration.a == 0 && params.max_acceleration.b == 0 && params.max_acceleration.c == 0 &&
                params.max_nullspace_velocity == 0 && params.max_nullspace_acceleration == 0);
    }

    inline bool jointImpedanceParamsIsDefault(const victor_hardware_interface::JointImpedanceParameters& params)
    {
        return (params.joint_stiffness.joint_1 == 0 && params.joint_stiffness.joint_2 == 0 && params.joint_stiffness.joint_3 == 0 && params.joint_stiffness.joint_4 == 0 &&
                params.joint_stiffness.joint_5 == 0 && params.joint_stiffness.joint_6 == 0 && params.joint_stiffness.joint_7 == 0 &&
                params.joint_damping.joint_1 == 0 && params.joint_damping.joint_2 == 0 && params.joint_damping.joint_3 == 0 && params.joint_damping.joint_4 == 0 &&
                params.joint_damping.joint_5 == 0 && params.joint_damping.joint_6 == 0 && params.joint_damping.joint_7 == 0);
    }

    inline bool cartesianImpedanceParamsIsDefault(const victor_hardware_interface::CartesianImpedanceParameters& params)
    {
        return (params.cartesian_stiffness.x == 0 && params.cartesian_stiffness.y == 0 && params.cartesian_stiffness.z == 0 && 
                params.cartesian_stiffness.a == 0 && params.cartesian_stiffness.b == 0 && params.cartesian_stiffness.c == 0 &&
                params.cartesian_damping.x == 0 && params.cartesian_damping.y == 0 && params.cartesian_damping.z == 0 && 
                params.cartesian_damping.a == 0 && params.cartesian_damping.b == 0 && params.cartesian_damping.c == 0 &&
                params.nullspace_stiffness == 0 && params.nullspace_damping == 0);
    }

    inline bool cartesianControlModeLimitsIsDefault(const victor_hardware_interface::CartesianControlModeLimits& params)
    {
        return (params.max_path_deviation.x == 0 && params.max_path_deviation.y == 0 && params.max_path_deviation.z == 0 && 
                params.max_path_deviation.a == 0 && params.max_path_deviation.b == 0 && params.max_path_deviation.c == 0 &&
                params.max_cartesian_velocity.x == 0 && params.max_cartesian_velocity.y == 0 && params.max_cartesian_velocity.z == 0 && 
                params.max_cartesian_velocity.a == 0 && params.max_cartesian_velocity.b == 0 && params.max_cartesian_velocity.c == 0 &&
                params.max_control_force.x == 0 && params.max_control_force.y == 0 && params.max_control_force.z == 0 && 
                params.max_control_force.a == 0 && params.max_control_force.b == 0 && params.max_control_force.c == 0 &&
                params.stop_on_max_control_force == false);
    }

    inline victor_hardware_interface::ControlModeCommand mergeControlModeCommand(const victor_hardware_interface::ControlModeStatus& active_control_mode, 
                                                                                 const victor_hardware_interface::ControlModeCommand& new_control_mode)
    {
        /*****************************************************************************************************************************************
        This function is a help function for the callback function of setting a new control mode(setControlModeCallBack). It copies the parameters
         of the old control mode to the new one, and updates relevant parameters with the parameters of the new control mode.
        
        Parameters updated in each control mode:
        JOINT_POSITION: joint_path_execution_params
        CARTESIAN_POSE: cartesian_path_execution_params
        JOINT_IMPEDANCE: joint_impedance_params, joint_path_execution_params
        CARTESIAN_IMPEDANCE: cartesian_impedance_params, cartesian_control_mode_limits, cartesian_path_execution_params
        *****************************************************************************************************************************************/

        victor_hardware_interface::ControlModeCommand merged_control_mode;
        // Copy the old over
        merged_control_mode.joint_path_execution_params = active_control_mode.joint_path_execution_params;
        merged_control_mode.joint_impedance_params = active_control_mode.joint_impedance_params;
        merged_control_mode.cartesian_impedance_params = active_control_mode.cartesian_impedance_params;
        merged_control_mode.cartesian_control_mode_limits = active_control_mode.cartesian_control_mode_limits;
        merged_control_mode.cartesian_path_execution_params = active_control_mode.cartesian_path_execution_params;
        // Copy manadatory members
        merged_control_mode.control_mode = new_control_mode.control_mode;
        // Copy mode-dependant members
        if (new_control_mode.control_mode == victor_hardware_interface::ControlModeCommand::JOINT_IMPEDANCE)
        {
            // From the new
            merged_control_mode.joint_path_execution_params = new_control_mode.joint_path_execution_params;
            merged_control_mode.joint_impedance_params = new_control_mode.joint_impedance_params;

            if(!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian impedance parameters are specified but ignored in JOINT_IMPEDANCE mode.");
            }
            if(!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian control mode limits are specified but ignored in JOINT_IMPEDANCE mode.");
            }
            if(!cartesianPathExecutionParamsIsDefault(new_control_mode.cartesian_path_execution_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian path execution parameters are specified but ignored in JOINT_IMPEDANCE mode.");
            }
        }
        else if (new_control_mode.control_mode == victor_hardware_interface::ControlModeCommand::CARTESIAN_IMPEDANCE)
        {
            // From the new
            merged_control_mode.cartesian_impedance_params = new_control_mode.cartesian_impedance_params;
            merged_control_mode.cartesian_control_mode_limits = new_control_mode.cartesian_control_mode_limits;
            merged_control_mode.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;

            if(!jointPathExecutionParamsIsDefault(new_control_mode.joint_path_execution_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The joint path execution parameters are specified but ignored in CASRTESIAN_IMPEDANCE mode.");
            }
            if(!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The joint impedance parameters are specified but ignored in CASRTESIAN_IMPEDANCE mode.");
            }
        }
        else if (new_control_mode.control_mode == victor_hardware_interface::ControlModeCommand::JOINT_POSITION)
        {
            // From the new
            merged_control_mode.joint_path_execution_params = new_control_mode.joint_path_execution_params;
            
            if(!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The joint impedance parameters are specified but ignored in JOINT_POSITION mode.");
            }
            if(!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian impedance parameters are specified but ignored in JOINT_POSITION mode.");
            }
            if(!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian control mode limits are specified but ignored in JOINT_POSITION mode.");
            }
            if(!cartesianPathExecutionParamsIsDefault(new_control_mode.cartesian_path_execution_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian path execution parameters are specified but ignored in JOINT_POSITION mode.");
            }
        }
        else if (new_control_mode.control_mode == victor_hardware_interface::ControlModeCommand::CARTESIAN_POSE)
        {
            // From the new
            merged_control_mode.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;

            if(!jointPathExecutionParamsIsDefault(new_control_mode.joint_path_execution_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The joint path execution parameters are specified but ignored in CARTESIAN_POSE mode.");
            }
            if(!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The joint impedance parameters are specified but ignored in CARTESIAN_POSE mode.");
            }
            if(!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian impedance parameters are specified but ignored in CARTESIAN_POSE mode.");
            }
            if(!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits))
            {
                ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian control mode limits are specified but ignored in CARTESIAN_POSE mode.");
            }
        }
        else
        {
            ROS_INFO_STREAM_NAMED(ros::this_node::getName(),"Invalid control mode: " << new_control_mode.control_mode << ".");
            assert(false);
        }
        return merged_control_mode;
    }
}
