from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get ROS_IP from environment or use default
    zivid_optical_frame = np.array(
        [[-0.577093,  0.631466, -0.517894,  1256.015625], 
        [ 0.812396,  0.508733, -0.284963,  85.490105], 
        [ 0.083526, -0.585185, -0.806587,  1898.157349], 
        [ 0.000000,  0.000000,  0.000000,  1.000000]]
    )
    zivid_optical_frame[:3, 3] /= 1000.0
    calibrated_mat_flat = zivid_optical_frame.flatten().tolist()
    # mocap_vec = [-0.6229922806800796, 0.6858175240681051, -0.6757562460396909,
    #             0.010570186931784202 ,-0.0035824702370211847 ,0.06951835956903875 ,
    #             0.9975182377972135]
    mocap_vec = [
        -0.0863941559344028, -0.5809860227759043, -0.671128621638247,
        0.014424292690806702, 0.002140815451877831, 0.09426630849713975, 0.9954402140620467
    ]
    calibrated_mat_str = str(calibrated_mat_flat)
    mocap_vec_str = str(mocap_vec)
    
    # Declare launch arguments
    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='VictorTeleopRealRobotProfile',
        description='Profile for VR control'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='false',
        description='Use simulator or real robot'
    )
    
    return LaunchDescription(
        [
            # Declare arguments
            profile_arg,
            use_sim_arg,

            # 1. Start publishing camera pose and mocap transformations, 
            # For some reason this needs to be before the main ROS stack goes online. 
            Node(
                package="arm_robots",
                executable="camera_pose_publisher.py",
                name="camera_pose_publisher",
                output="screen",
                namespace="victor",
                parameters=[
                    {"calibrated_mat": calibrated_mat_str},
                    {"mocap_vec": mocap_vec_str}
                ]
            ),

            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    str(os.path.join(
                        get_package_share_directory("lightweight_vicon_bridge"),
                        "launch/vicon_bridge.launch"
                    ))
                ),
            ),

            # Node(
            #     package="arm_zivid",
            #     executable="arm_zivid_ros_node.py",
            #     name="zivid_publisher",
            # ),
            
            # 2. Start rossim (MoveIt demo with RViz and simulator)
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'victor_moveit_config', 'demo.launch.py', 
                    'use_rviz:=true', 
                    ['use_simulator:=', LaunchConfiguration('use_simulator')],
                ],
                output='screen'
            ),
            
        ]
    )