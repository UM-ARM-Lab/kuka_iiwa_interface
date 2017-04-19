#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <iiwa_robot_controllers/FRIState.h>
#include <iiwa_robot_controllers/FRICommand.h>
#include <iiwa_robot_controllers/iiwa_robot_config.hpp>
#include <linux_fri_client/friUdpConnection.h>
#include <linux_fri_client/friClientData.h>
#include <linux_fri_client/friLBRClient.h>
#include <linux_fri_client/FRIMessages.pb.h>
#include <arc_utilities/eigen_helpers.hpp>

class FRIStatePositionTorqueShim
{
protected:

    enum ARM_CONTROL_MODE {POSITION=1, TORQUE_POSITION=2, UNKNOWN=0};

    ARM_CONTROL_MODE arm_mode_;
    std::vector<std::string> joint_names_;
    std::map<std::string, iiwa_robot_controllers::JointLimits> joint_limits_;

    bool has_active_command_;
    iiwa_robot_controllers::FRICommand active_command_;

    bool has_current_feedback_;
    iiwa_robot_controllers::FRIState current_feedback_;

    ros::NodeHandle nh_;
    ros::Publisher feedback_pub_;
    ros::Subscriber command_sub_;
    ros::CallbackQueue ros_callback_queue_;
    std::thread ros_callback_thread_;
    std::mutex command_setting_lock_;
    std::mutex feedback_setting_lock_;

public:

    FRIStatePositionTorqueShim(ros::NodeHandle& nh,
                               const std::string& feedback_topic,
                               const std::string& command_topic,
                               const std::map<std::string, iiwa_robot_controllers::JointLimits>& joint_limits) : nh_(nh)
    {
        assert(joint_limits.size() == 7);
        nh_.setCallbackQueue(&ros_callback_queue_);
        joint_names_ = arc_helpers::GetKeys(joint_limits);
        joint_limits_ = joint_limits;
        has_active_command_ = false;
        has_current_feedback_ = false;
        arm_mode_ = UNKNOWN;
        // Setup publishers and subscribers
        feedback_pub_ = nh_.advertise<iiwa_robot_controllers::FRIState>(feedback_topic, 1, false);
        command_sub_ = nh_.subscribe(command_topic, 1, &FRIStatePositionTorqueShim::CommandCallback, this);
        // Spin up the callback helper thread.
        ros_callback_thread_ = std::thread(std::bind(&FRIStatePositionTorqueShim::ROSCallbackThread, this));
    }

    void ROSCallbackThread()
    {
        const double timeout = 0.001;
        while (nh_.ok())
        {
            ros_callback_queue_.callAvailable(ros::WallDuration(timeout));
            feedback_setting_lock_.lock();
            const bool has_current_feedback = has_current_feedback_;
            iiwa_robot_controllers::FRIState current_feedback = current_feedback_;
            feedback_setting_lock_.unlock();
            if (has_current_feedback)
            {
                feedback_pub_.publish(current_feedback);
            }
        }
    }

    std::string PrintSessionState(const KUKA::FRI::ESessionState& state) const
    {
        if (state == KUKA::FRI::IDLE)
        {
            return "IDLE";
        }
        else if (state == KUKA::FRI::MONITORING_WAIT)
        {
            return "MONITORING_WAIT";
        }
        else if (state == KUKA::FRI::MONITORING_READY)
        {
            return "MONITORING_READY";
        }
        else if (state == KUKA::FRI::COMMANDING_WAIT)
        {
            return "COMMANDING_WAIT";
        }
        else if (state == KUKA::FRI::COMMANDING_ACTIVE)
        {
            return "COMMANDING_ACTIVE";
        }
        else
        {
            return "UNKNOWN";
        }
    }

    void Loop(const std::string& fri_address, const int fri_port)
    {
        while (ros::ok())
        {
            ROS_INFO("Resetting has_active_command_ to false, as FRI connection is being rebuilt");
            has_active_command_ = false;
            ROS_INFO("Initializing FRI connection to %s on port %i...", fri_address.c_str(), fri_port);
            KUKA::FRI::UdpConnection robot_connection;
            const bool fri_connected = robot_connection.open(fri_port, fri_address.c_str());
//            std::chrono::time_point<std::chrono::high_resolution_clock> previous_cycle_end_time = std::chrono::high_resolution_clock::now();
            if (fri_connected)
            {
                ROS_INFO("FRI connection started");
                KUKA::FRI::ClientData robot_data(7);
                robot_data.expectedMonitorMsgID = 0x245142;
                robot_data.commandMsg.header.messageIdentifier = 0x34001;
                while (ros::ok())
                {
                    // Handle FRI
                    if (!robot_connection.isOpen())
                    {
                        ROS_ERROR("FRI connection is not open!");
                        break;
                    }
                    // **************************************************************************
                    // Receive and decode new monitoring message
                    // **************************************************************************
                    int size = robot_connection.receive(robot_data.receiveBuffer, KUKA::FRI::FRI_MONITOR_MSG_MAX_SIZE);
                    if (size <= 0)
                    {
                        ROS_ERROR("Failure receiving FRI message, got %d", size);
                        break;
                    }
                    if (!robot_data.decoder.decode(robot_data.receiveBuffer, size))
                    {
                        ROS_ERROR("Failure decoding FRI message");
                        break;
                    }
                    // Check message type (so that our wrappers match)
                    if (robot_data.expectedMonitorMsgID != robot_data.monitoringMsg.header.messageIdentifier)
                    {
                        ROS_ERROR("Error: incompatible IDs for received message (got: %d expected %d)!", (int)robot_data.monitoringMsg.header.messageIdentifier, (int)robot_data.expectedMonitorMsgID);
                        break;
                    }
                    // Reset commmand message before callbacks
                    robot_data.resetCommandMessage();
                    // Handle FRI tasks
                    const KUKA::FRI::ESessionState current_state = (KUKA::FRI::ESessionState)robot_data.monitoringMsg.connectionInfo.sessionState;
                    if (robot_data.lastState != current_state)
                    {
                        const std::string last_mode_str = PrintSessionState(robot_data.lastState);
                        const std::string new_mode_str = PrintSessionState(current_state);
                        ROS_INFO("Switched from state %s to state %s", last_mode_str.c_str(), new_mode_str.c_str());
                        robot_data.lastState = current_state;
                    }
                    // Set the expected control mode
                    const KUKA::FRI::EClientCommandMode current_client_command_mode = GetClientCommandMode(robot_data.monitoringMsg);
                    if (current_client_command_mode == KUKA::FRI::POSITION)
                    {
                        arm_mode_ = POSITION;
                    }
                    else if (current_client_command_mode == KUKA::FRI::TORQUE)
                    {
                        arm_mode_ = TORQUE_POSITION;
                    }
                    else
                    {
                        arm_mode_ = UNKNOWN;
                    }
                    // Main mode switch
                    if (current_state == KUKA::FRI::MONITORING_WAIT || current_state == KUKA::FRI::MONITORING_READY)
                    {
                        PublishFRIState(robot_data.monitoringMsg);
                    }
                    else if (current_state == KUKA::FRI::COMMANDING_WAIT)
                    {
                        PublishFRIState(robot_data.monitoringMsg);
                        ApplyFRICommand(robot_data.monitoringMsg, robot_data.commandMsg);
                    }
                    else if (current_state == KUKA::FRI::COMMANDING_ACTIVE)
                    {
                        PublishFRIState(robot_data.monitoringMsg);
                        ApplyFRICommand(robot_data.monitoringMsg, robot_data.commandMsg);
                    }
                    // **************************************************************************
                    // Encode and send command message
                    // **************************************************************************
                    robot_data.lastSendCounter++;
                    // check if its time to send an answer
                    if (robot_data.lastSendCounter >= robot_data.monitoringMsg.connectionInfo.receiveMultiplier)
                    {
                        robot_data.lastSendCounter = 0;

                        // set sequence counters
                        robot_data.commandMsg.header.sequenceCounter = robot_data.sequenceCounter++;
                        robot_data.commandMsg.header.reflectedSequenceCounter = robot_data.monitoringMsg.header.sequenceCounter;

                        if (!robot_data.encoder.encode(robot_data.sendBuffer, size))
                        {
                            ROS_ERROR("Failed to encode FRI message");
                            break;
                        }

                        if (!robot_connection.send(robot_data.sendBuffer, size))
                        {
                            ROS_ERROR("Failed sending FRI message");
                            break;
                        }
                    }
//                    std::chrono::time_point<std::chrono::high_resolution_clock> cycle_end_time = std::chrono::high_resolution_clock::now();
//                    std::chrono::duration<double> cycle_time(cycle_end_time - previous_cycle_end_time);
//                    previous_cycle_end_time = cycle_end_time;
//                    printf("Actual cycle time %5.4f\n", cycle_time.count());
                    // Process callbacks
                    ros::spinOnce();
                }
                ROS_INFO("Closing FRI session...");
                if (robot_connection.isOpen())
                {
                    robot_connection.close();
                }
                ROS_INFO("FRI session closed");
            }
            else
            {
                ROS_ERROR("Failed to connect FRI interface");
            }
        }
    }

    //******************************************************************************
    // THIS IS DESIRED CYCLE TIME, NOT ACTUAL CYCLE TIME!
    double GetSampleTime(const FRIMonitoringMessage& monitor_msg) const
    {
        return monitor_msg.connectionInfo.sendPeriod * 0.001;
    }

    //******************************************************************************
    KUKA::FRI::ESessionState GetSessionState(const FRIMonitoringMessage& monitor_msg) const
    {
        return (KUKA::FRI::ESessionState)monitor_msg.connectionInfo.sessionState;
    }

    //******************************************************************************
    KUKA::FRI::EConnectionQuality GetConnectionQuality(const FRIMonitoringMessage& monitor_msg) const
    {
        return (KUKA::FRI::EConnectionQuality)monitor_msg.connectionInfo.quality;
    }

    //******************************************************************************
    KUKA::FRI::ESafetyState GetSafetyState(const FRIMonitoringMessage& monitor_msg) const
    {
        return (KUKA::FRI::ESafetyState)monitor_msg.robotInfo.safetyState;
    }

    //******************************************************************************
    KUKA::FRI::EOperationMode GetOperationMode(const FRIMonitoringMessage& monitor_msg) const
    {
        return (KUKA::FRI::EOperationMode)monitor_msg.robotInfo.operationMode;
    }

    //******************************************************************************
    KUKA::FRI::EDriveState GetDriveState(const FRIMonitoringMessage& monitor_msg) const
    {
        tRepeatedIntArguments *values = (tRepeatedIntArguments *)monitor_msg.robotInfo.driveState.arg;
        int firstState = (int)values->value[0];
        for (int i = 1; i < 7; i++)
        {
            int state = (int)values->value[i];
            if (state != firstState)
            {
                return KUKA::FRI::TRANSITIONING;
            }
        }
        return (KUKA::FRI::EDriveState)firstState;
    }

    //********************************************************************************
    KUKA::FRI::EOverlayType GetOverlayType(const FRIMonitoringMessage& monitor_msg) const
    {
        return (KUKA::FRI::EOverlayType)monitor_msg.ipoData.overlayType;
    }

    //********************************************************************************
    KUKA::FRI::EClientCommandMode GetClientCommandMode(const FRIMonitoringMessage& monitor_msg) const
    {
        return (KUKA::FRI::EClientCommandMode)monitor_msg.ipoData.clientCommandMode;
    }

    //******************************************************************************
    KUKA::FRI::EControlMode GetControlMode(const FRIMonitoringMessage& monitor_msg) const
    {
        return (KUKA::FRI::EControlMode)monitor_msg.robotInfo.controlMode;
    }

    //******************************************************************************
    ros::Time GetTimestamp(const FRIMonitoringMessage& monitor_msg) const
    {
        ros::Time timestamp;
        timestamp.sec = monitor_msg.monitorData.timestamp.sec;
        timestamp.nsec = monitor_msg.monitorData.timestamp.nanosec;
        return timestamp;
    }

    std::vector<double> RepeatedArgumentsToVector(const tRepeatedDoubleArguments* values) const
    {
        return std::vector<double>{values->value[0], values->value[1], values->value[2], values->value[3], values->value[4], values->value[5], values->value[6]};
    }

    //******************************************************************************
    const std::vector<double> GetMeasuredJointPosition(const FRIMonitoringMessage& monitor_msg) const
    {
        return RepeatedArgumentsToVector((tRepeatedDoubleArguments*)monitor_msg.monitorData.measuredJointPosition.value.arg);
    }

    //******************************************************************************
    const std::vector<double> GetCommandedJointPosition(const FRIMonitoringMessage& monitor_msg) const
    {
        return RepeatedArgumentsToVector((tRepeatedDoubleArguments*)monitor_msg.monitorData.commandedJointPosition.value.arg);
    }

    //******************************************************************************
    const std::vector<double> GetMeasuredTorque(const FRIMonitoringMessage& monitor_msg) const
    {
        return RepeatedArgumentsToVector((tRepeatedDoubleArguments*)monitor_msg.monitorData.measuredTorque.value.arg);
    }

    //******************************************************************************
    const std::vector<double> GetCommandedTorque(const FRIMonitoringMessage& monitor_msg) const
    {
        return RepeatedArgumentsToVector((tRepeatedDoubleArguments*)monitor_msg.monitorData.commandedTorque.value.arg);
    }

    //******************************************************************************
    const std::vector<double> GetExternalTorque(const FRIMonitoringMessage& monitor_msg) const
    {
        return RepeatedArgumentsToVector((tRepeatedDoubleArguments*)monitor_msg.monitorData.externalTorque.value.arg);
    }

    //******************************************************************************
    const std::vector<double> GetInterpolatedJointPosition(const FRIMonitoringMessage& monitor_msg) const
    {
        if (monitor_msg.ipoData.has_jointPosition)
        {
            return RepeatedArgumentsToVector((tRepeatedDoubleArguments*)monitor_msg.ipoData.jointPosition.value.arg);
        }
        else
        {
            return std::vector<double>();
        }
    }

    //******************************************************************************
    double GetTrackingPerformance(const FRIMonitoringMessage& monitor_msg) const
    {
        if (monitor_msg.ipoData.has_trackingPerformance)
        {
            return monitor_msg.ipoData.trackingPerformance;
        }
        else
        {
             return 0.0;
        }
    }

    //******************************************************************************
    void SetJointPosition(const std::vector<double>& joint_positions, FRICommandMessage& command_msg)
    {
        command_msg.has_commandData = true;
        command_msg.commandData.has_jointPosition = true;
        tRepeatedDoubleArguments *dest = (tRepeatedDoubleArguments*)command_msg.commandData.jointPosition.value.arg;
        memcpy(dest->value, joint_positions.data(), 7 * sizeof(double));
    }

    //******************************************************************************
    void SetWrench(const std::vector<double>& wrench, FRICommandMessage& command_msg)
    {
        command_msg.has_commandData = true;
        command_msg.commandData.has_cartesianWrenchFeedForward = true;
        double *dest = command_msg.commandData.cartesianWrenchFeedForward.element;
        memcpy(dest, wrench.data(), 6 * sizeof(double));
    }

    //******************************************************************************
    void SetTorque(const std::vector<double>& joint_torques, FRICommandMessage& command_msg)
    {
        command_msg.has_commandData = true;
        command_msg.commandData.has_jointTorque = true;
        tRepeatedDoubleArguments *dest = (tRepeatedDoubleArguments*)command_msg.commandData.jointTorque.value.arg;
        memcpy(dest->value, joint_torques.data(), 7 * sizeof(double));
    }

    void PublishFRIState(const FRIMonitoringMessage& monitor_msg)
    {
        iiwa_robot_controllers::FRIState feedback_msg;
        feedback_msg.header.stamp = GetTimestamp(monitor_msg);
        feedback_msg.joint_name = joint_names_;
        feedback_msg.joint_position_measured = GetMeasuredJointPosition(monitor_msg);
        feedback_msg.joint_position_commanded = GetCommandedJointPosition(monitor_msg);
        feedback_msg.joint_position_interpolated = GetInterpolatedJointPosition(monitor_msg);
        feedback_msg.joint_torque_measured = GetMeasuredTorque(monitor_msg);
        feedback_msg.joint_torque_external = GetExternalTorque(monitor_msg);
        feedback_msg.joint_torque_commanded = GetCommandedTorque(monitor_msg);
        feedback_msg.tracking_performance = GetTrackingPerformance(monitor_msg);
        feedback_msg.cycle_time = GetSampleTime(monitor_msg);
        feedback_msg.client_command_mode = (uint8_t)GetClientCommandMode(monitor_msg);
        feedback_msg.control_mode = (uint8_t)GetControlMode(monitor_msg);
        feedback_msg.drive_state = (uint8_t)GetDriveState(monitor_msg);
        feedback_msg.operating_mode = (uint8_t)GetOperationMode(monitor_msg);
        feedback_msg.safety_state = (uint8_t)GetSafetyState(monitor_msg);
        feedback_msg.connection_quality = (uint8_t)GetConnectionQuality(monitor_msg);
        feedback_msg.fri_connection_state = (uint8_t)GetSessionState(monitor_msg);
        feedback_setting_lock_.lock();
        has_current_feedback_ = true;
        std::swap(current_feedback_, feedback_msg);
        feedback_setting_lock_.unlock();
    }

    void ApplyFRICommand(const FRIMonitoringMessage& monitor_msg, FRICommandMessage& command_msg)
    {
        command_setting_lock_.lock();
        const bool has_active_command = has_active_command_;
        const uint8_t command_control_mode = active_command_.mode;
        const std::vector<double> command = active_command_.joint_command;
        command_setting_lock_.unlock();
        const KUKA::FRI::EClientCommandMode control_mode = GetClientCommandMode(monitor_msg);
        const std::vector<double> current_measured_joint_positions = GetMeasuredJointPosition(monitor_msg);
        const std::vector<double> current_commanded_joint_positions = GetCommandedJointPosition(monitor_msg);
        const double time_delta = GetSampleTime(monitor_msg);
        if (control_mode == KUKA::FRI::POSITION)
        {
            if (has_active_command && (command_control_mode == iiwa_robot_controllers::FRICommand::POSITION))
            {
                //std::cout << "POSITION MODE - ACTIVE FRI COMMAND MODE" << std::endl;
                const std::vector<double> interpolated_joint_target = InterpolatePositionTarget(current_measured_joint_positions, command, time_delta);
                printf("CURRENT AT %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f\nCURRENT CMD %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f\nCURRENT GOTO %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f, %+5.4f\n", current_measured_joint_positions[0], current_measured_joint_positions[1], current_measured_joint_positions[2], current_measured_joint_positions[3], current_measured_joint_positions[4], current_measured_joint_positions[5], current_measured_joint_positions[6], current_commanded_joint_positions[0], current_commanded_joint_positions[1], current_commanded_joint_positions[2], current_commanded_joint_positions[3], current_commanded_joint_positions[4], current_commanded_joint_positions[5], current_commanded_joint_positions[6], interpolated_joint_target[0], interpolated_joint_target[1], interpolated_joint_target[2], interpolated_joint_target[3], interpolated_joint_target[4], interpolated_joint_target[5], interpolated_joint_target[6]);
                SetJointPosition(interpolated_joint_target, command_msg);
            }
            else if (has_active_command && (command_control_mode == iiwa_robot_controllers::FRICommand::TORQUE))
            {
                //std::cout << "POSITION MODE - INVALID FRI COMMAND MODE" << std::endl;
                SetJointPosition(current_commanded_joint_positions, command_msg);
            }
            else
            {
                //std::cout << "POSITION MODE - INACTIVE FRI COMMAND MODE" << std::endl;
                SetJointPosition(current_commanded_joint_positions, command_msg);
            }
        }
        else if (control_mode == KUKA::FRI::TORQUE)
        {
            if (has_active_command && (command_control_mode == iiwa_robot_controllers::FRICommand::TORQUE))
            {
                //std::cout << "TORQUE MODE - ACTIVE FRI COMMAND MODE" << std::endl;
                SetJointPosition(current_measured_joint_positions, command_msg);
                SetTorque(command, command_msg);
            }
            else if (has_active_command && (command_control_mode == iiwa_robot_controllers::FRICommand::POSITION))
            {
                //std::cout << "TORQUE MODE - FAKE POSITION FRI COMMAND MODE" << std::endl;
                const std::vector<double> interpolated_joint_target = InterpolatePositionTarget(current_measured_joint_positions, command, time_delta);
                SetJointPosition(interpolated_joint_target, command_msg);
                const std::vector<double> zero_torque(7, 0.0);
                SetTorque(zero_torque, command_msg);
            }
            else
            {
                //std::cout << "TORQUE MODE - INACTIVE FRI COMMAND MODE" << std::endl;
                SetJointPosition(current_commanded_joint_positions, command_msg);
                const std::vector<double> zero_torque(7, 0.0);
                SetTorque(zero_torque, command_msg);
            }
        }
        else
        {
            assert(false);
        }
    }

    std::vector<double> InterpolatePositionTarget(const std::vector<double>& current_joint_positions, const std::vector<double>& target_joint_positions, const double time_delta) const
    {
        ROS_ASSERT_MSG(time_delta <= 0.01, "Time delta exceeded safe threshold");
        // Compute delta
        const std::vector<double> delta = EigenHelpers::Sub(target_joint_positions, current_joint_positions);
        // Convert delta to necessary velocity/step
        const std::vector<double> velocity_per_step = EigenHelpers::Divide(delta, time_delta);
        // Limit velocity to limits
        std::vector<double> limited_velocity_per_step(velocity_per_step.size(), 0.0);
        for (size_t idx = 0; idx < joint_names_.size(); idx++)
        {
            // Get the name of the joint
            const std::string& joint_name = joint_names_[idx];
            const double velocity_val = velocity_per_step[idx];
            // Get the limit for the joint
            const auto limit_found_itr = joint_limits_.find(joint_name);
            // Limit the joint command
            assert(limit_found_itr != joint_limits_.end());
            const double max_velocity = limit_found_itr->second.MaxVelocity();
            const double limited_velocity = arc_helpers::ClampValue(velocity_val, -max_velocity, max_velocity);
            limited_velocity_per_step[idx] = limited_velocity;
        }
        // Convert into limited delta
        const std::vector<double> limited_delta = EigenHelpers::Multiply(limited_velocity_per_step, time_delta);
        // Get interpolated target
        const std::vector<double> interpolated_target = EigenHelpers::Add(current_joint_positions, limited_delta);
        return interpolated_target;
    }

    void CommandCallback(iiwa_robot_controllers::FRICommand command)
    {
        if (command.mode == iiwa_robot_controllers::FRICommand::POSITION || command.mode == iiwa_robot_controllers::FRICommand::TORQUE)
        {
            if (command.joint_name.size() == command.joint_command.size())
            {
                // Push the command into a map
                std::map<std::string, double> command_map;
                for (size_t idx = 0; idx < command.joint_name.size(); idx++)
                {
                    const std::string& name = command.joint_name[idx];
                    const double command_val = command.joint_command[idx];
                    command_map[name] = command_val;
                }
                // Extract the joint commands in order
                iiwa_robot_controllers::FRICommand ordered_command;
                ordered_command.mode = command.mode;
                ordered_command.joint_name = joint_names_;
                ordered_command.joint_command.resize(joint_names_.size(), 0.0);
                bool command_valid = true;
                for (size_t idx = 0; idx < joint_names_.size(); idx++)
                {
                    // Get the name of the joint
                    const std::string& joint_name = joint_names_[idx];
                    // Get the commanded value
                    const auto found_itr = command_map.find(joint_name);
                    if (found_itr != command_map.end())
                    {
                        const double command_val = found_itr->second;
                        // Get the limit for the joint
                        const auto limit_found_itr = joint_limits_.find(joint_name);
                        // Limit the joint command
                        assert(limit_found_itr != joint_limits_.end());
                        if (command.mode == iiwa_robot_controllers::FRICommand::POSITION)
                        {
                            const double limited_position = arc_helpers::ClampValueAndWarn(command_val, limit_found_itr->second.MinPosition(), limit_found_itr->second.MaxPosition());
                            ordered_command.joint_command[idx] = limited_position;
                        }
                        else if (command.mode == iiwa_robot_controllers::FRICommand::TORQUE)
                        {
                            const double max_torque = limit_found_itr->second.MaxEffort();
                            const double limited_torque = arc_helpers::ClampValueAndWarn(command_val, -max_torque, max_torque);
                            ordered_command.joint_command[idx] = limited_torque;
                        }
                    }
                    else
                    {
                        ROS_ERROR("Invalid FRI command: joint %s missing", joint_name.c_str());
                        command_valid = false;
                    }
                }
                if (command_valid == true)
                {
                    if ((arm_mode_ == POSITION) && (ordered_command.mode == iiwa_robot_controllers::FRICommand::POSITION))
                    {
                        //"Received valid FRI position command");
                        command_setting_lock_.lock();
                        has_active_command_ = true;
                        std::swap(active_command_, ordered_command);
                        command_setting_lock_.unlock();
                    }
                    else if ((arm_mode_ == TORQUE_POSITION) && (ordered_command.mode == iiwa_robot_controllers::FRICommand::POSITION))
                    {
                        //ROS_DEBUG("Received valid FRI position (in torque mode) command");
                        command_setting_lock_.lock();
                        has_active_command_ = true;
                        std::swap(active_command_, ordered_command);
                        command_setting_lock_.unlock();
                    }
                    else if ((arm_mode_ == TORQUE_POSITION) && (ordered_command.mode == iiwa_robot_controllers::FRICommand::TORQUE))
                    {
                        //ROS_DEBUG("Received valid FRI torque command");
                        command_setting_lock_.lock();
                        has_active_command_ = true;
                        std::swap(active_command_, ordered_command);
                        command_setting_lock_.unlock();
                    }
//                    else
//                    {
//                        ROS_WARN("Ignored valid FRI command mode %i, does not match arm mode %i", ordered_command.mode, arm_mode_);
//                    }
                }
            }
            else
            {
                ROS_ERROR("Invalid FRI command: %zu names, %zu commands", command.joint_name.size(), command.joint_command.size());
            }
        }
        else
        {
            ROS_ERROR("Invalid FRI command mode");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_FRI_shim");
    ROS_INFO("Starting iiwa_FRI_shim...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_FEEDBACK_TOPIC = "iiwa_FRI_state";
    const std::string DEFAULT_COMMAND_TOPIC = "iiwa_FRI_command";
    const std::string DEFAULT_JOINT_NAME_PREFIX = "iiwa";
    const std::string DEFAULT_FRI_ADDRESS = "192.170.10.2";
    const int DEFAULT_FRI_PORT = 30200;
    const double DEFAULT_POSITION_LIMIT_SCALING = 0.95;
    const double DEFAULT_VELOCITY_LIMIT_SCALING = 0.025;
    const double DEFAULT_TORQUE_LIMIT_SCALING = 0.1;
    std::string feedback_topic;
    std::string command_topic;
    std::string joint_name_prefix;
    std::string fri_address;
    int fri_port = DEFAULT_FRI_PORT;
    double position_limit_scaling = DEFAULT_POSITION_LIMIT_SCALING;
    double velocity_limit_scaling = DEFAULT_VELOCITY_LIMIT_SCALING;
    double torque_limit_scaling = DEFAULT_TORQUE_LIMIT_SCALING;
    nhp.param(std::string("command_topic"), command_topic, DEFAULT_COMMAND_TOPIC);
    nhp.param(std::string("feedback_topic"), feedback_topic, DEFAULT_FEEDBACK_TOPIC);
    nhp.param(std::string("joint_name_prefix"), joint_name_prefix, DEFAULT_JOINT_NAME_PREFIX);
    nhp.param(std::string("fri_address"), fri_address, DEFAULT_FRI_ADDRESS);
    nhp.param(std::string("fri_port"), fri_port, DEFAULT_FRI_PORT);
    nhp.param(std::string("position_limit_scaling"), position_limit_scaling, DEFAULT_POSITION_LIMIT_SCALING);
    nhp.param(std::string("velocity_limit_scaling"), velocity_limit_scaling, DEFAULT_VELOCITY_LIMIT_SCALING);
    nhp.param(std::string("torque_limit_scaling"), torque_limit_scaling, DEFAULT_TORQUE_LIMIT_SCALING);
    const double real_position_limit_scaling = arc_helpers::ClampValueAndWarn(position_limit_scaling, 0.0, 1.0);
    const double real_velocity_limit_scaling = arc_helpers::ClampValueAndWarn(velocity_limit_scaling, 0.0, 1.0);
    const double real_torque_limit_scaling = arc_helpers::ClampValueAndWarn(torque_limit_scaling, 0.0, 1.0);
    // Joint limits
    const std::map<std::string, iiwa_robot_controllers::JointLimits> joint_limits = iiwa_robot_controllers::GetArmLimits(joint_name_prefix, real_position_limit_scaling, real_velocity_limit_scaling, real_torque_limit_scaling);
    // Assemble the controller
    FRIStatePositionTorqueShim shim(nh, feedback_topic, command_topic, joint_limits);
    ROS_INFO("...startup complete");
    shim.Loop(fri_address, fri_port);
    return 0;
}
