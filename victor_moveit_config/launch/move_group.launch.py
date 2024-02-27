from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("victor", package_name="victor_moveit_config").to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }


    debug_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[ moveit_config.to_dict(), move_group_configuration],
        output="screen",
        # namespace="victor",
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": ":0"},
    )
    ld.add_action(debug_node)
    return ld