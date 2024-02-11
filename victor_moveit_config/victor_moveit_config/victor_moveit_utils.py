from ament_index_python import get_package_share_directory
from moveit.planning import MoveItPy

from moveit_configs_utils import MoveItConfigsBuilder


def load_moveit_robot():
    moveit_config_dict = load_moveit_config_dict()
    robot = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
    return robot


def load_moveit_config_dict():
    moveit_config_builder = MoveItConfigsBuilder('victor')
    moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("victor_moveit_config") + "/config/moveit_cpp.yaml")
    moveit_config = moveit_config_builder.to_moveit_configs()
    moveit_config_dict = moveit_config.to_dict()

    return moveit_config_dict
