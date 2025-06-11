from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("use_simulator", default_value=False))

    # moveit_config = MoveItConfigsBuilder("victor", package_name="victor_moveit_config").to_moveit_configs()
    moveit_config = (
        MoveItConfigsBuilder("victor", package_name="victor_moveit_config")
        .robot_description(file_path="config/victor.urdf.xacro", 
                          mappings={"fake_hardware": LaunchConfiguration("use_simulator")})
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    return ld
