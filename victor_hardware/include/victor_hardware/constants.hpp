#include <string>

const std::string DEFAULT_CARTESIAN_CONTROL_FRAME = "base";
const double DEFAULT_SET_CONTROL_MODE_TIMEOUT = 2.5;  // seconds

// Default ROS topic / service names
const std::string DEFAULT_MOTION_COMMAND_TOPIC("motion_command");
const std::string DEFAULT_MOTION_STATUS_TOPIC("motion_status");
const std::string DEFAULT_CONTROL_MODE_STATUS_TOPIC("control_mode_status");
const std::string DEFAULT_SET_CONTROL_MODE_SERVICE("set_control_mode_service");
const std::string DEFAULT_GET_CONTROL_MODE_SERVICE("get_control_mode_service");
const std::string DEFAULT_GRIPPER_COMMAND_TOPIC("gripper_command");
const std::string DEFAULT_GRIPPER_STATUS_TOPIC("gripper_status");

// Default LCM parameters
const std::string DEFAULT_SEND_LCM_URL("udp://10.10.10.11:30000");
const std::string DEFAULT_RECV_LCM_URL("udp://10.10.10.108:30001");
const std::string DEFAULT_MOTION_COMMAND_CHANNEL("motion_command");
const std::string DEFAULT_MOTION_STATUS_CHANNEL("motion_status");
const std::string DEFAULT_CONTROL_MODE_COMMAND_CHANNEL("control_mode_command");
const std::string DEFAULT_CONTROL_MODE_STATUS_CHANNEL("control_mode_status");
const std::string DEFAULT_GRIPPER_COMMAND_CHANNEL("gripper_command");
const std::string DEFAULT_GRIPPER_STATUS_CHANNEL("gripper_status");

const std::string CARTESIAN_CONTROL_FRAME_PARAM("cartesian_control_frame");
const std::string SET_CONTROL_MODE_TIMEOUT_PARAM("set_control_mode_timeout");
const std::string MOTION_COMMAND_TOPIC_PARAM("motion_command_topic");
const std::string MOTION_STATUS_TOPIC_PARAM("motion_status_topic");
const std::string CONTROL_MODE_STATUS_TOPIC_PARAM("control_mode_status_topic");
const std::string GET_CONTROL_MODE_SERVICE_PARAM("get_control_mode_service");
const std::string SET_CONTROL_MODE_SERVICE_PARAM("set_control_mode_service");
const std::string GRIPPER_COMMAND_TOPIC_PARAM("gripper_command_topic");
const std::string GRIPPER_STATUS_TOPIC_PARAM("gripper_status_topic");
const std::string SEND_LCM_URL_PARAM("send_lcm_url");
const std::string RECV_LCM_URL_PARAM("recv_lcm_url");
const std::string MOTION_COMMAND_CHANNEL_PARAM("motion_command_channel");
const std::string MOTION_STATUS_CHANNEL_PARAM("motion_status_channel");
const std::string CONTROL_MODE_COMMAND_CHANNEL_PARAM("control_mode_command_channel");
const std::string CONTROL_MODE_STATUS_CHANNEL_PARAM("control_mode_status_channel");
const std::string GRIPPER_COMMAND_CHANNEL_PARAM("gripper_command_channel");
const std::string GRIPPER_STATUS_CHANNEL_PARAM("gripper_status_channel");
