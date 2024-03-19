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
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("victor_command_gui", default_value=True))
    # Note that changing these from the default requires matching changes in the Java code
    ld.add_action(DeclareLaunchArgument("left_arm_recv_url", default_value="udp://10.10.10.169:30002"))
    ld.add_action(DeclareLaunchArgument("right_arm_recv_url", default_value="udp://10.10.10.169:30001"))

    ld.add_action(
        Node(
            package="victor_python",
            executable="victor_command_gui.py",
            condition=IfCondition(LaunchConfiguration("victor_command_gui")),
        )
    )

    ld.add_action(
        Node(
            package="victor_hardware",
            executable="iiwa_lcm_bridge_node",
            name="left_arm_lcm_bridge_node",
            output="screen",
            namespace="victor/left_arm",
            parameters=[
                {"send_lcm_url": "udp://10.10.10.12:30000"},
                {"recv_lcm_url": LaunchConfiguration("left_arm_recv_url")},
                {"cartesian_pose_frame": "victor_left_arm_world_frame_kuka"},
            ],
        )
    )

    ld.add_action(
        Node(
            package="victor_hardware",
            executable="iiwa_lcm_bridge_node",
            name="right_arm_lcm_bridge_node",
            output="screen",
            namespace="victor/right_arm",
            parameters=[
                {"send_lcm_url": "udp://10.10.10.11:30000"},
                {"recv_lcm_url": LaunchConfiguration("right_arm_recv_url")},
                {"cartesian_pose_frame": "victor_right_arm_world_frame_kuka"},
            ],
        )
    )

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
    return ld
