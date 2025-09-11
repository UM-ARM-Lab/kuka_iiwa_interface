from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, NotSubstitution
import os


def generate_launch_description():
    # Get the workspace path
    ros_ws_path = os.environ.get('ROS_WS_PATH', '/home/houhd/code/robot_tool_2025S/utils/ros_ws')

    # Declare launch argument
    use_simulator = LaunchConfiguration('use_simulator', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        # Declare use_simulator argument
        DeclareLaunchArgument(
            'use_simulator',
            default_value='true',
            description='Whether to use the simulator'
        ),

        # 1. Start rossim (MoveIt demo with RViz and simulator)
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'victor_moveit_config', 'demo.launch.py',
                ['use_rviz:=', use_rviz],
                ['use_simulator:=', use_simulator]
            ],
            output='screen'
        ),

        # 3. Start arm_zivid_ros_node.py immediately (only when not using simulator)
        ExecuteProcess(
            cmd=[
                'python3', 'src/arm_zivid/arm_zivid/arm_zivid_ros_node.py',
                # '--settings_yml', os.path.join(ros_ws_path, 'config', 'zivid2_11Hz_Engine_2D3D_Final.yml')
                # "--task", "collect",
                '--settings_yml', os.path.join(ros_ws_path, 'config', 'zivid2_20Hz_Engine_2D3D_Final_downsample.yml'),
                # '--settings_yml', os.path.join(ros_ws_path, 'config', 'zivid2_20Hz_Engine_3D_Final_very_bad_downsample.yml')
                # '--settings_yml', os.path.join(ros_ws_path, 'config', 'zivid2_20Hz_Engine_3D_Final_very_bad_downsample_v3.yml')
                # "-r", "~/datasets/robotool_runs", 
                # "-n", "$(date +%Y%m%d_%H%M%S)", 
                # "--pub_pc",
            ],
            output='screen',
            cwd=ros_ws_path,
            condition=IfCondition(NotSubstitution(use_simulator))
        ),

        # 2. Start policy server after 10 second delay
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', 'src/kuka_iiwa_interface/victor_python/victor_python/victor_policy_server.py'],
                    output='screen',
                    cwd=ros_ws_path
                )
            ]
        )
    ])
