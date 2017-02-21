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
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <iiwa_robot_controllers/VelocityCommand.h>
#include <iiwa_robot_controllers/iiwa_robot_config.hpp>
#include <iiwa_msgs/JointVelocity.h>

class VelocityShim
{
protected:

    std::vector<std::string> joint_names_;
    std::map<std::string, iiwa_robot_controllers::JointLimits> joint_limits_;

    ros::NodeHandle nh_;
    ros::Publisher velocity_command_pub_;
    ros::Subscriber velocity_target_sub_;

public:

    VelocityShim(ros::NodeHandle& nh,
                                        const std::string& velocity_target_topic,
                                        const std::string& velocity_command_topic,
                                        const std::map<std::string, iiwa_robot_controllers::JointLimits> joint_limits) : nh_(nh)
    {
        assert(joint_limits.size() == 7);
        joint_names_ = arc_helpers::GetKeys(joint_limits);
        joint_limits_ = joint_limits;
        // Setup publishers and subscribers
        velocity_command_pub_ = nh_.advertise<iiwa_msgs::JointVelocity>(velocity_command_topic, 1, false);
        velocity_target_sub_ = nh_.subscribe(velocity_target_topic, 1, &VelocityShim::ConvertToIiwaMessage, this);
    }

    void Loop(const double cycle_rate)
    {
        ros::Rate spin_rate(cycle_rate);
        while (ros::ok())
        {
            // Process callbacks
            ros::spinOnce();
            // Spin
            spin_rate.sleep();
        }
    }

    void ConvertToIiwaMessage(iiwa_robot_controllers::VelocityCommand velocity_command)
    {
        if (velocity_command.name.size() == velocity_command.command.size())
        {
            // Push the command into a map
            std::map<std::string, double> command_map;
            for (size_t idx = 0; idx < velocity_command.name.size(); idx++)
            {
                const std::string& name = velocity_command.name[idx];
                const double command = velocity_command.command[idx];
                command_map[name] = command;
            }
            // Extract the joint commands in order
            std::vector<double> target_velocities(joint_names_.size(), 0.0);
            bool command_valid = true;
            for (size_t idx = 0; idx < joint_names_.size(); idx++)
            {
                // Get the name of the joint
                const std::string& joint_name = joint_names_[idx];
                // Get the commanded value
                const auto found_itr = command_map.find(joint_name);
                if (found_itr != command_map.end())
                {
                    const double velocity = found_itr->second;
                    // Get the limit for the joint
                    const auto limit_found_itr = joint_limits_.find(joint_name);
                    // Limit the joint command
                    assert(limit_found_itr != joint_limits_.end());
                    const double limit = limit_found_itr->second.MaxVelocity();
                    const double limited_velocity = arc_helpers::ClampValue(velocity, -limit, limit);
                    target_velocities[idx] = limited_velocity;
                }
                else
                {
                    ROS_ERROR("Invalid velocity command: joint %s missing", joint_name.c_str());
                    command_valid = false;
                }
            }
            if (command_valid == true)
            {
                iiwa_msgs::JointVelocity iiwa_msg;
                iiwa_msg.header.stamp = ros::Time::now();
                iiwa_msg.velocity.a1 = (float)target_velocities[0];
                iiwa_msg.velocity.a2 = (float)target_velocities[1];
                iiwa_msg.velocity.a3 = (float)target_velocities[2];
                iiwa_msg.velocity.a4 = (float)target_velocities[3];
                iiwa_msg.velocity.a5 = (float)target_velocities[4];
                iiwa_msg.velocity.a6 = (float)target_velocities[5];
                iiwa_msg.velocity.a7 = (float)target_velocities[6];
                velocity_command_pub_.publish(iiwa_msg);
            }
        }
        else
        {
            ROS_ERROR("Invalid velocity command: %zu names, %zu commands", velocity_command.name.size(), velocity_command.command.size());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_stack_velocity_shim");
    ROS_INFO("Starting iiwa_stack_velocity_shim...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_VELOCITY_TARGET_TOPIC = "joint_command_velocity";
    const std::string DEFAULT_VELOCITY_COMMAND_TOPIC = "/iiwa/command/JointVelocity";
    const std::string DEFAULT_JOINT_NAME_PREFIX = "iiwa";
    const double DEFAULT_CYCLE_RATE = 500.0; //25.0;
    std::string velocity_target_topic;
    std::string velocity_command_topic;
    std::string joint_name_prefix;
    double cycle_rate = DEFAULT_CYCLE_RATE;
    nhp.param(std::string("velocity_command_topic"), velocity_command_topic, DEFAULT_VELOCITY_COMMAND_TOPIC);
    nhp.param(std::string("velocity_target_topic"), velocity_target_topic, DEFAULT_VELOCITY_TARGET_TOPIC);
    nhp.param(std::string("joint_name_prefix"), joint_name_prefix, DEFAULT_JOINT_NAME_PREFIX);
    nhp.param(std::string("cycle_rate"), cycle_rate, DEFAULT_CYCLE_RATE);
    // Joint limits
    const std::map<std::string, iiwa_robot_controllers::JointLimits> joint_limits = iiwa_robot_controllers::GetArmLimits(joint_name_prefix);
    // Assemble the controller
    VelocityShim shim(nh, velocity_target_topic, velocity_command_topic, joint_limits);
    ROS_INFO("...startup complete");
    shim.Loop(cycle_rate);
    return 0;
}
