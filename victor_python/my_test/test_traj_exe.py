from threading import Thread

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arm_utilities.transformation_helper import get_vec7_from_transform
from victor_python.victor import Victor
import sys
import copy

# sys.path.append("/home/zixuanh/UMD")
# from visualization.plot import plot_seg_fig, plot_pointclouds


def create_cfg():
    from omegaconf import OmegaConf
    cfg = OmegaConf.create()   

    # 
    cfg.initial_joint_pos = [2.1638144841967337, -0.4226588050996085, 2.0798249613706896, 
                             1.5527786438822437, -2.3819865555514106, 0.7058400474683474, 0.6201394769293104]
    #
    cfg.initial_pose = [ 0.69327585,  0.49556933,  0.76353298, -0.64432928,  0.05128085,
        0.76152394,  0.04786799] # x,y,z, qx,qy,qz,qw
   
    cfg.radius = 0.175 # unit: m



    # only use robot hand to apporoximate the bolt position no accurate probably
    cfg.bolts_position = [0.68382,0.5296,0.77402]    # version1
    cfg.bolts_position = [0.54305,0.51534,0.77402]    # version2

    



    #table config
    cfg.table_width = 0.6 # unit: m 60cm total length
    cfg.table_length = 1.365 # unit: m 136.5cm total length
    cfg.mocap_table_left_upper_conner_position = [1.1,0.72,0.66] # from mocap 

    #iron config
    cfg.iron_width =  0.076 # unit: m 7.6cm total length
    cfg.iron_height = 0.076 # unit: m 7.6cm total length
    cfg.left_table2boltboard= 0.172 # unit: m 17.2cm, the distance from the left table/iron to the left side of bolt board 
    cfg.boltboard_width = 0.076 # unit: m 7.6cm total length
    cfg.boltboard_length = 0.152 # unit: m 15.2cm total length    
    cfg.boltboard_height = 0.00633 # unit: m 6.63mm total length
    cfg.upperboard_height = 0.022 # unit: m 2.2cm total length


    # other config
    x_when_robot_hand_touch_iron = 0.59755

    cfg.steps = 100

    return cfg 

def get_bolt_pos(cfg):
    # get the bolt position from mocap
    mocap_table_pos = cfg.mocap_table_left_upper_conner_position

    bolts_pos_x = mocap_table_pos[0] - cfg.table_width + cfg.iron_width/2
    bolts_pos_y = mocap_table_pos[1] - cfg.left_table2boltboard - cfg.boltboard_length/2
    bolts_pos_z = mocap_table_pos[2] + cfg.iron_height +  cfg.upperboard_height + cfg.boltboard_height 
    return [bolts_pos_x,bolts_pos_y,bolts_pos_z]



def cal_pose_from_bolt_pos(cfg):
    # cal the pose from bolt position
    pass

def degree_euler2qua_numpy(euler,order='xyz'):
    from scipy.spatial.transform import Rotation
    r = Rotation.from_euler(order, euler, degrees=True)
    qua = r.as_quat()
    return qua

def quat_mul(a, b):
    assert a.shape == b.shape
    shape = a.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 4)

    x1, y1, z1, w1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    x2, y2, z2, w2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    quat = np.stack([x, y, z, w], axis=-1) 

    return quat

def radians2degree(radians):
    return radians * 180 / np.pi

def circle_traj(cfg,socket_pos_np,cur_pos_np,screw_direction='cw',delta_angle=0.1,time_steps=1):
    """

    """
    # radius = np.linalg.norm(cur_pos_np[:2] - socket_pos_np[:2])
    radius = cfg.radius
    
    current_angle_randians = np.arctan2(cur_pos_np[1]-socket_pos_np[1],cur_pos_np[0]-socket_pos_np[0])
    current_angle_degree = radians2degree(current_angle_randians)
    if screw_direction == 'cw':
        target_angle = current_angle_randians -  delta_angle * time_steps
                
    elif screw_direction == 'ccw':
        target_angle = current_angle_randians +  delta_angle  * time_steps
    
    target_pos_x = socket_pos_np[0] + radius * np.cos(target_angle)
    target_pos_y = socket_pos_np[1] + radius * np.sin(target_angle)
    target_pos_z = 0.76353298

    # target orientation

    defaut_orientation =  np.array([[ -0.7071068, 0, 0.7071068, 0 ]])  # Example quaternion

    target_angle = np.arctan2(target_pos_y-socket_pos_np[1],target_pos_x-socket_pos_np[0])
    target_angle_degree = radians2degree(target_angle)

    # if screw_direction == 'cw':
    delta_qua = degree_euler2qua_numpy([0,0,target_angle_degree],order='xyz').reshape(1,4)
    # elif screw_direction == 'ccw':
        # delta_qua = degree_euler2qua_numpy([0,0,-target_angle_degree],order='xyz').reshape(1,4)

    quaternion = quat_mul(delta_qua,defaut_orientation)
    target_pisition = np.array([target_pos_x,target_pos_y,target_pos_z]).reshape(1,3)
    target_pose = np.concatenate([target_pisition,quaternion],axis=1)
    return target_pose


def set_target_pose(target_pose_np, current_pose):

    target_pose = copy.deepcopy( current_pose)
    
    target_pose.translation.x = target_pose_np[0]
    target_pose.translation.y = target_pose_np[1]
    target_pose.translation.z = target_pose_np[2]

    target_pose.rotation.x = target_pose_np[3]
    target_pose.rotation.y = target_pose_np[4]
    target_pose.rotation.z = target_pose_np[5]
    target_pose.rotation.w = target_pose_np[6]

    return target_pose




def main():
    cfg = create_cfg()
    bolts_pos = cfg.bolts_position

    rclpy.init()
    node = Node("create_collision_scene")
    victor = Victor(node)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin)
    spin_thread.start()
    victor.clear_all_collision_objects()
    victor.add_collision_box("wall_back", position=(-0.4, -0., 1.15), quat_xyzw=(0, 0, 0, 1),
                             size=(0.2, 2, 1.7))
    victor.add_collision_box("wall_left", position=(0.4, 1.1, 1), quat_xyzw=(0, 0, 0, 1),
                             size=(1.6, 0.1, 2))
    victor.add_collision_box("wall_right", position=(0.4, -1.1, 1), quat_xyzw=(0, 0, 0, 1),
                             size=(1.6, 0.1, 2))
    victor.add_collision_box("table", position=(0.8, -0., 0.5), quat_xyzw=(0, 0, 0, 1),
                             size=(cfg.table_width, cfg.table_length, 0.3))
    victor.add_collision_box("iron", position=(0.54, 0.5, 0.725), quat_xyzw=(0, 0, 0, 1),
                             size=(0.076,1 , 0.15))
    
    res = victor.set_controller("joint_impedance_trajectory_controller")


    print(victor.get_left_joint_positions())
    
    # v1
    # init_joints = [0.6380733901465206, -0.15606636651061342, 2.4770401185063657, 1.1271591959754055, -1.664421981347256, 1.5973075894988655, 0.7805908113797119]
    # init_pose = [0.44124, -0.14164, 0.62189, -0.5042, 0.46525, 0.49146, 0.53647]
 
    victor.plan_to_joint_config(cfg.initial_joint_pos, "left_arm")
    # victor.plan_to_pose(init_pose, "right_arm", "victor_right_tool0")
    print(victor.get_joint_cmd_dict())
    res = victor.set_controller("impedance_controller")

    ideal_poses = []
    actual_poses = []

    turning_direction = 'cw'

    for i in range(cfg.steps):
        current_pose = victor.get_link_pose("victor_left_tool0")
        print("current_pose",current_pose)
        current_pose_np = np.array(get_vec7_from_transform(current_pose)) # dim 7
        cur_angle = np.arctan2(current_pose_np[1]-bolts_pos[1],current_pose_np[0]-bolts_pos[0])

         
        target_pose_np = circle_traj(cfg,bolts_pos,current_pose_np,screw_direction=turning_direction,delta_angle=0.25).squeeze()
        print("target_pose_np",target_pose_np)

        if cur_angle < -0.5:
            turning_direction = 'ccw'
        elif cur_angle > 0.5:
            turning_direction = 'cw'




        target_pose = victor.get_link_pose("victor_left_tool0")

        target_pose.translation.x = target_pose_np[0]
        target_pose.translation.y = target_pose_np[1]
        target_pose.translation.z = target_pose_np[2]

        target_pose.rotation.x = target_pose_np[3]
        target_pose.rotation.y = target_pose_np[4]
        target_pose.rotation.z = target_pose_np[5]
        target_pose.rotation.w = target_pose_np[6]

        # target_pose = set_target_pose(target_pose_np, current_pose)
       
        victor.move_to_pose("left_arm", target_pose)
        

    # current_pose = victor.get_link_pose("victor_left_tool0")

    # ideal_pos = np.array(get_vec7_from_transform(current_pose)[:3])

    # delta = np.array([
    #     [0, -0.1, 0],
    #     [0.1, 0, 0],
    #     [0, 0.1, 0],
    #     [0.1, 0, 0],
    # ])
    # for i in range(5):
    #     for x in delta:
    #         tool_pose = victor.get_link_pose("victor_left_tool0")
    #         ideal_pos = ideal_pos + x
    #         tool_pose.translation.x = ideal_pos[0]
    #         tool_pose.translation.y = ideal_pos[1]
    #         tool_pose.translation.z = ideal_pos[2]

    #         ideal_poses.append(ideal_pos)
    #         victor.move_to_pose("left_arm", tool_pose)
    #         new_pose = victor.get_link_pose("victor_left_tool0")
    #         new_pos = get_vec7_from_transform(new_pose)[:3]
    #         actual_poses.append(new_pos)
    #         print("Execution error ", np.linalg.norm(new_pos - ideal_pos))
    # plot_pointclouds([np.stack(ideal_poses), np.stack(actual_poses)]).show()
    tool_pose = victor.get_link_pose("victor_left_tool0")
    tool_pose.translation.x += 0.1
    victor.move_to_pose("left_arm", tool_pose)
    # test cartesian mode
    motion_status = victor.left.motion_status.get()
    print(motion_status.measured_cartesian_pose_abc)
    print(motion_status.commanded_cartesian_pose_abc)
    print(motion_status.measured_cartesian_pose)
    print(motion_status.commanded_cartesian_pose)


if __name__ == '__main__':
    main()