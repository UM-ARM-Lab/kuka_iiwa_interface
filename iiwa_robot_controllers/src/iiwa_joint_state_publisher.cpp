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
#include <iiwa_robot_controllers/FRIState.h>
#include <iiwa_robot_controllers/iiwa_robot_config.hpp>
#include <arc_utilities/vector_math.hpp>

class IiwaJointStatePublisher
{
protected:

    std::vector<std::string> joint_names_;

    uint32_t velocity_filter_depth_;
    std::vector<iiwa_robot_controllers::FRIState> state_msg_queue_;

    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber fri_state_sub_;

public:

    IiwaJointStatePublisher(ros::NodeHandle& nh,
                                        const std::string& fri_state_topic,
                                        const std::string& joint_state_topic,
                                        const uint32_t velocity_filter_depth,
                                        const std::vector<std::string>& joint_names) : nh_(nh)
    {
        assert(joint_names.size() == 7);
        joint_names_ = joint_names;
        velocity_filter_depth_ = velocity_filter_depth;
        state_msg_queue_.clear();
        // Setup publishers and subscribers
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic, 1, false);
        fri_state_sub_ = nh_.subscribe(fri_state_topic, 1, &IiwaJointStatePublisher::FRIStateCallback, this);
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

    void FRIStateCallback(iiwa_robot_controllers::FRIState fri_state)
    {
        if (state_msg_queue_.size() < velocity_filter_depth_)
        {
            state_msg_queue_.push_back(fri_state);
        }
        else
        {
            state_msg_queue_.erase(state_msg_queue_.begin(), state_msg_queue_.begin() + 1);
            state_msg_queue_.push_back(fri_state);
        }
        // Publish joint_states
        if (state_msg_queue_.size() < velocity_filter_depth_)
        {
            ROS_INFO("Waiting to fill state message queue to publish joint states");
        }
        else
        {
            PublishJointStates(state_msg_queue_);
        }
    }

    void PublishJointStates(const std::vector<iiwa_robot_controllers::FRIState>& state_msgs)
    {
        const iiwa_robot_controllers::FRIState& oldest_state = state_msgs.front();
        const std::vector<double>& oldest_positions = oldest_state.joint_position_measured;
        const iiwa_robot_controllers::FRIState& newest_state = state_msgs.back();
        const std::vector<double>& newest_positions = newest_state.joint_position_measured;
        const ros::Duration time_interval = newest_state.header.stamp - oldest_state.header.stamp;
        const std::vector<double> joint_position_delta = EigenHelpers::Sub(newest_positions, oldest_positions);
        const std::vector<double> joint_velocities = EigenHelpers::Divide(joint_position_delta, time_interval.toSec());
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = newest_state.header.stamp;
        joint_state.name = newest_state.joint_name;
        joint_state.position = newest_state.joint_position_measured;
        joint_state.effort = newest_state.joint_torque_measured;
        joint_state.velocity = joint_velocities;
        joint_state_pub_.publish(joint_state);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iiwa_joint_state_publisher");
    ROS_INFO("Starting iiwa_joint_state_publisher...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string DEFAULT_FRI_STATE_TOPIC = "iiwa_FRI_state";
    const std::string DEFAULT_JOINT_STATE_TOPIC = "joint_states";
    const std::string DEFAULT_JOINT_NAME_PREFIX = "iiwa";
    const double DEFAULT_CYCLE_RATE = 1000.0; //25.0;
    const int32_t DEFAULT_VELOCITY_FILTER_DEPTH = 5;
    std::string fri_state_topic;
    std::string joint_state_topic;
    std::string joint_name_prefix;
    double cycle_rate = DEFAULT_CYCLE_RATE;
    int32_t velocity_filter_depth = DEFAULT_VELOCITY_FILTER_DEPTH;
    nhp.param(std::string("velocity_command_topic"), joint_state_topic, DEFAULT_JOINT_STATE_TOPIC);
    nhp.param(std::string("velocity_target_topic"), fri_state_topic, DEFAULT_FRI_STATE_TOPIC);
    nhp.param(std::string("joint_name_prefix"), joint_name_prefix, DEFAULT_JOINT_NAME_PREFIX);
    nhp.param(std::string("cycle_rate"), cycle_rate, DEFAULT_CYCLE_RATE);
    nhp.param(std::string("velocity_filter_depth"), velocity_filter_depth, DEFAULT_VELOCITY_FILTER_DEPTH);
    uint32_t real_velocity_filter_depth = 1u;
    if (velocity_filter_depth < 1)
    {
        ROS_WARN("Velocity filter depth cannot be less than 1, set to 1");
    }
    else
    {
        real_velocity_filter_depth = (uint32_t)velocity_filter_depth;
        ROS_INFO("Velocity filter depth set to %u", real_velocity_filter_depth);
    }
    // Joint limits
    const std::vector<std::string> joint_names = iiwa_robot_controllers::GetJointNames(joint_name_prefix);
    // Assemble the controller
    IiwaJointStatePublisher JSP(nh, fri_state_topic, joint_state_topic, real_velocity_filter_depth, joint_names);
    ROS_INFO("...startup complete");
    JSP.Loop(cycle_rate);
    return 0;
}
