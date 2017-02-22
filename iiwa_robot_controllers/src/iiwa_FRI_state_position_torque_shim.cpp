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
#include <ros/ros.h>
#include <iiwa_robot_controllers/FRIState.h>
#include <iiwa_robot_controllers/FRICommand.h>
#include <iiwa_robot_controllers/iiwa_robot_config.hpp>
#include <linux_fri_client/friUdpConnection.h>
#include <linux_fri_client/friClientData.h>
#include <linux_fri_client/friLBRClient.h>
#include <linux_fri_client/FRIMessages.pb.h>

class FRIStatePositionTorqueShim
{
protected:

    std::vector<std::string> joint_names_;
    std::map<std::string, iiwa_robot_controllers::JointLimits> joint_limits_;

    bool has_active_command_;
    iiwa_robot_controllers::FRICommand active_command_;

    ros::NodeHandle nh_;
    ros::Publisher feedback_pub_;
    ros::Subscriber command_sub_;

public:

    FRIStatePositionTorqueShim(ros::NodeHandle& nh,
                                        const std::string& feedback_topic,
                                        const std::string& command_topic,
                                        const std::map<std::string, iiwa_robot_controllers::JointLimits>& joint_limits) : nh_(nh)
    {
        assert(joint_limits.size() == 7);
        joint_names_ = arc_helpers::GetKeys(joint_limits);
        joint_limits_ = joint_limits;
        has_active_command_ = false;
        // Setup publishers and subscribers
        feedback_pub_ = nh_.advertise<iiwa_robot_controllers::FRIState>(feedback_topic, 1, false);
        command_sub_ = nh_.subscribe(command_topic, 1, &FRIStatePositionTorqueShim::CommandCallback, this);
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
            ROS_INFO("Initializing FRI connection...");
            KUKA::FRI::UdpConnection robot_connection;
            const bool fri_connected = robot_connection.open(fri_port, fri_address.c_str());
            if (fri_connected)
            {
                ROS_INFO("FRI connection established");
                KUKA::FRI::ClientData robot_data(7);
                robot_data.expectedMonitorMsgID = 0x245142;
                robot_data.commandMsg.header.messageIdentifier = 0x34001;
                while (ros::ok())
                {
                    // Process callbacks
                    ros::spinOnce();
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
        feedback_pub_.publish(feedback_msg);
    }

    void ApplyFRICommand(const FRIMonitoringMessage& monitor_msg, FRICommandMessage& command_msg)
    {
        const KUKA::FRI::EClientCommandMode control_mode = GetClientCommandMode(monitor_msg);
        const std::vector<double> current_measured_joint_positions = GetMeasuredJointPosition(monitor_msg);
        if (control_mode == KUKA::FRI::POSITION)
        {
            if (has_active_command_ && (active_command_.mode == iiwa_robot_controllers::FRICommand::POSITION))
            {
                std::cout << "POSITION MODE - ACTIVE FRI COMMAND MODE" << std::endl;
                SetJointPosition(active_command_.joint_command, command_msg);
            }
            else if (has_active_command_ && (active_command_.mode == iiwa_robot_controllers::FRICommand::TORQUE))
            {
                std::cout << "POSITION MODE - INVALID FRI COMMAND MODE" << std::endl;
                SetJointPosition(current_measured_joint_positions, command_msg);
            }
            else
            {
                std::cout << "POSITION MODE - INACTIVE FRI COMMAND MODE" << std::endl;
                SetJointPosition(current_measured_joint_positions, command_msg);
            }
        }
        else if (control_mode == KUKA::FRI::TORQUE)
        {
            if (has_active_command_ && (active_command_.mode == iiwa_robot_controllers::FRICommand::TORQUE))
            {
                std::cout << "TORQUE MODE - ACTIVE FRI COMMAND MODE" << std::endl;
                SetJointPosition(current_measured_joint_positions, command_msg);
                SetTorque(active_command_.joint_command, command_msg);
            }
            else if (has_active_command_ && (active_command_.mode == iiwa_robot_controllers::FRICommand::POSITION))
            {
                std::cout << "TORQUE MODE - FAKE POSITION FRI COMMAND MODE" << std::endl;
                SetJointPosition(active_command_.joint_command, command_msg);
                const std::vector<double> zero_torque(7, 0.0);
                SetTorque(zero_torque, command_msg);
            }
            else
            {
                std::cout << "TORQUE MODE - INACTIVE FRI COMMAND MODE" << std::endl;
                SetJointPosition(current_measured_joint_positions, command_msg);
                const std::vector<double> zero_torque(7, 0.0);
                SetTorque(zero_torque, command_msg);
            }
        }
        else
        {
            assert(false);
        }
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
                    ROS_DEBUG("Received valid FRI command");
                    has_active_command_ = true;
                    active_command_ = ordered_command;
                }
                else
                {
                    has_active_command_ = false;
                }
            }
            else
            {
                ROS_ERROR("Invalid FRI command: %zu names, %zu commands", command.joint_name.size(), command.joint_command.size());
                has_active_command_ = false;
            }
        }
        else
        {
            ROS_ERROR("Invalid FRI command mode");
            has_active_command_ = false;
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
    const std::string DEFAULT_FRI_ADDRESS = "10.68.1.1";
    const double DEFAULT_CYCLE_RATE = 1000.0;
    const int DEFAULT_FRI_PORT = 30200;
    std::string feedback_topic;
    std::string command_topic;
    std::string joint_name_prefix;
    std::string fri_address;
    double cycle_rate = DEFAULT_CYCLE_RATE;
    int fri_port = DEFAULT_FRI_PORT;
    nhp.param(std::string("command_topic"), command_topic, DEFAULT_COMMAND_TOPIC);
    nhp.param(std::string("feedback_topic"), feedback_topic, DEFAULT_FEEDBACK_TOPIC);
    nhp.param(std::string("joint_name_prefix"), joint_name_prefix, DEFAULT_JOINT_NAME_PREFIX);
    nhp.param(std::string("cycle_rate"), cycle_rate, DEFAULT_CYCLE_RATE);
    nhp.param(std::string("fri_address"), fri_address, DEFAULT_FRI_ADDRESS);
    nhp.param(std::string("fri_port"), fri_port, DEFAULT_FRI_PORT);
    // Joint limits
    const std::map<std::string, iiwa_robot_controllers::JointLimits> joint_limits = iiwa_robot_controllers::GetArmLimits(joint_name_prefix);
    // Assemble the controller
    FRIStatePositionTorqueShim shim(nh, feedback_topic, command_topic, joint_limits);
    ROS_INFO("...startup complete");
    shim.Loop(fri_address, fri_port);
    return 0;
}
