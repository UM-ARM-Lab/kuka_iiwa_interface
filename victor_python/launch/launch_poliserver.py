from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():
    # Get the workspace path
    ros_ws_path = os.environ.get('ROS_WS_PATH', '/home/houhd/code/robot_tool_2025S/utils/ros_ws')
    
    return LaunchDescription(
        [
            # 1. Start rossim (MoveIt demo with RViz and simulator)
            ExecuteProcess(
                cmd=['ros2', 'launch', 'victor_moveit_config', 'demo.launch.py', 'use_rviz:=true', 'use_simulator:=false'],
                output='screen'
            ),
            
            # 2. Start policy server after 10 second delay
            TimerAction(
                period=10.0,
                actions=[
                    ExecuteProcess(
                        cmd=['python3', 'src/kuka_iiwa_interface/victor_python/victor_python/victor_policy_server.py'],
                        output='screen',
                        cwd=ros_ws_path
                    )
                ]
            )
        ]
    )
