from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("victor", package_name="victor_moveit_config").to_moveit_configs()

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("victor_command_gui", default_value=False))

    ld.add_action(
        Node(
            package="victor_python",
            executable="victor_command_gui.py",
            condition=IfCondition(LaunchConfiguration("victor_command_gui")),
        )
    )

    ld.add_action(
        Node(
            package="joy",
            executable="joy_node",
            name="xbox_joystick",
            namespace="victor",
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="victor_python",
            executable="robotiq_grippers_joystick_node.py",
            name="robotiq_grippers_joystick_node",
            output="screen",
            namespace="victor",
            parameters=[
                {
                    "enable_finger_open_close_control": True,
                    "enable_scissor_open_close_control": True,
                }
            ]
        )
    )

    ld.add_action(
        Node(
            package="victor_python",
            executable="arm_wrench_republisher.py",
            name="arm_wrench_republisher",
            output="screen",
            namespace="victor",
        )
    )

    # ld.add_action(
    #     Node(
    #         package="victor_python",
    #         executable="arm_wrench_republisher.py",
    #         namespace="victor",
    #     )
    # )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="rqt_reconfigure",
            executable="rqt_reconfigure",
        )
    )

    # Spawncontrollers
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers_inactive.launch.py"),
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py"),
            ),
        )
    )

    ld.add_action(
        Node(
            package="rqt_controller_manager",
            executable="rqt_controller_manager",
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py"),
            ),
        )
    )

    return ld
