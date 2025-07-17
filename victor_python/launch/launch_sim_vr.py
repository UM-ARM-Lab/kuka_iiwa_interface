from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Get ROS_IP from environment or use default
    ros_ip = os.environ.get('ROS_IP', '0.0.0.0')
    
    # Declare launch arguments
    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='VictorTeleopSimCapTrackerProfile',
        description='Profile for VR control'
    )
    
    return LaunchDescription(
        [
            # Declare arguments
            profile_arg,
            
            # 1. Start rossim (MoveIt demo with RViz and simulator)
            ExecuteProcess(
                cmd=['ros2', 'launch', 'victor_moveit_config', 'demo.launch.py', 'use_rviz:=true', 'use_simulator:=false'],
                output='screen'
            ),
            
            # 2. Start ros-unity teleop endpoint (starts immediately)
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                emulate_tty=True,
                parameters=[{"ROS_IP": ros_ip}],
                output='screen'
            ),
            
            # 3. Start VR teleop script after 10 second delay
            TimerAction(
                period=10.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'python3', 
                            'src/kuka_iiwa_interface/victor_python/victor_python/victor_vr_teleop.py', 
                            '--profile', 
                            LaunchConfiguration('profile')
                        ],
                        output='screen',
                        cwd='/home/houhd/code/robot_tool_2025S/utils/ros_ws'
                    )
                ]
            )
        ]
    )
