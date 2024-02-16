import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("victor", package_name="victor_moveit_config").to_moveit_configs()

    servo_yaml = load_yaml("victor_moveit_config", "config/moveit_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # container = ComposableNodeContainer(
    #     name="moveit_servo_demo_container",
    #     namespace="/",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         # Example of launching Servo as a node component
    #         # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
    #         ComposableNode(
    #             package="moveit_servo",
    #             plugin="moveit_servo::ServoServer",
    #             name="servo_server",
    #             parameters=[
    #                 servo_params,
    #                 moveit_config.robot_description,
    #                 moveit_config.robot_description_semantic,
    #                 moveit_config.robot_description_kinematics,
    #             ],
    #         ),
    #         ComposableNode(
    #             package="moveit_servo",
    #             plugin="moveit_servo::JoyToServoPub",
    #             name="controller_to_servo_node",
    #         ),
    #         ComposableNode(
    #             package="joy",
    #             plugin="joy::Joy",
    #             name="joy_node",
    #         ),
    #     ],
    #     output="screen",
    # )

    # return LaunchDescription([container])

    servo_node = Node(
        name="moveit_servo",
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        arguments=["--ros-args", "--log-level", "moveit_servo:=debug"],
        output="screen",
    )

    return LaunchDescription([servo_node])
