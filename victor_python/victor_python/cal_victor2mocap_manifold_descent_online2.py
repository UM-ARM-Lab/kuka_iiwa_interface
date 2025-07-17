import matplotlib
matplotlib.use('Agg')

import sys
sys.path.append('./')
from arm_utilities.tf2wrapper import TF2Wrapper
from arm_utilities.transformation_helper import build_mat_from_transform, extract_from_matrix

from rclpy.node import Node
import rclpy
import numpy as np
import time
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
# from utilis.ros2_utils import RvizPub
import autograd.numpy as anp
import pymanopt
from pymanopt.manifolds import Euclidean, Product, SpecialOrthogonalGroup
from pymanopt.optimizers import SteepestDescent, TrustRegions
from scipy.spatial.transform import Rotation as Rot
from victor_python.victor import Victor
# from utilis.data_utilis import store_h5_dict

import matplotlib
matplotlib.use('Agg')


def main():
    rclpy.init()
    node = Node("tf_static_from_mocap_arm_2_victor_arm")
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin)
    spin_thread.start()
    tfwrapper = TF2Wrapper(node)
    
    # rivz_pub = RvizPub(node,camera_frame='mocap_world',base_frame='victor_root', marker_color = 'blue', marker_scale=0.0135)
    victor = Victor(node)

    # calibrated_mat = np.eye(4)
    time.sleep(3)
    H_arm_in_mocap_msg = tfwrapper.get_transform(parent="mocap_world",child="mocap_right_arm_base_right_arm_base")
    H_arm_in_mocap = build_mat_from_transform(H_arm_in_mocap_msg)
    H_mocap_in_arm_mocap = np.linalg.inv(H_arm_in_mocap)
    print('calibrated_mat H_mocap_in_arm_mocap',H_mocap_in_arm_mocap)

    H_victor_in_arm_msg = tfwrapper.get_transform(parent="victor_right_arm_mount",child="victor_root")
    H_victor_in_arm_victor = build_mat_from_transform(H_victor_in_arm_msg)
    print('calibrated_mat H_victor_in_arm_victor',H_victor_in_arm_victor)


    # H_arm_victor_in_arm_mocap = np.eye(4)

    # p_victor_left_finger_b_link3_in_victor_frame = victor.get_link_position("left_finger_b_link3")
    # p_victor_left_finger_b_link3_in_victor_frame = np.array([0.75607,0.381604,0.754012])
    keep_runing = True

    data_dic = {
        'p_0_in_mocap':[],
        'p_0_in_arm_mocap':[], # p_0_in_arm_mocap_frames = H_mocap_in_arm_mocap @ p_0_in_mocap
        'p_0_in_victor':[], 
        'p_0_in_arm_victor':[], # p_0_in_arm_victor_frames = H_victor_arm_victor @ p_0_in_victor
    }
    p0_offset = np.array([0.00,0.01778,-0.047244])
    state_ = 'data_collection'

    R_optimal = np.array([[ 0.99944362,  0.02041487, -0.02637597],
                            [-0.01996605,  0.99965323,  0.0171693 ],
                            [ 0.02671733, -0.01663313,  0.99950464]])
    t_optimal = np.array([0.00606023, 0.00996432, 0.00197848])

    H_arm_mocap_in_arm_victor = np.eye(4)
    H_arm_mocap_in_arm_victor[:3,:3] = R_optimal
    H_arm_mocap_in_arm_victor[:3,3] = t_optimal

    H_arm_victor_in_arm_mocap = np.linalg.inv(H_arm_mocap_in_arm_victor)
    H_arm_mocap_in_mocap = np.linalg.inv(H_mocap_in_arm_mocap)


    H_victor_in_mocap =  H_arm_mocap_in_mocap @ H_arm_victor_in_arm_mocap @  H_victor_in_arm_victor
    pos, qua = extract_from_matrix(H_victor_in_mocap)
    print(f"ros2 run tf2_ros static_transform_publisher  {pos[0]} {pos[1]} {pos[2]} {qua[0]} {qua[1]} {qua[2]} {qua[3]} mocap_world victor_root")

    while keep_runing:
        # dis_mocap_dot2_board = 1.981 * 0.01 # m 
        # if state_ == 'data_collection':
        input('Move the iron chunk and the hand. Press <ENTER> to record data')
        H_p0_in_mocap_msg = tfwrapper.get_transform(parent="mocap_world",child="mocap_iron_chunk_3_iron_chunk_3")
        H_p0_in_mocap = build_mat_from_transform(H_p0_in_mocap_msg)
        H_p0_in_arm_mocap = H_mocap_in_arm_mocap @ H_p0_in_mocap
        p_0_in_mocap = extract_from_matrix(H_p0_in_mocap)[0]
        p_0_in_arm_mocap = extract_from_matrix(H_p0_in_arm_mocap)[0]
        print("p_0_in_mocap",p_0_in_mocap)
        data_dic['p_0_in_mocap'].append(p_0_in_mocap.copy())
        data_dic['p_0_in_arm_mocap'].append(p_0_in_arm_mocap.copy())

        H_p0_in_victor_msg = victor.get_link_pose("victor_right_tool0")
        H_p0_in_victor = build_mat_from_transform(H_p0_in_victor_msg)
        H_p0_in_arm_victor = H_victor_in_arm_victor @ H_p0_in_victor
        p0_in_victor = extract_from_matrix(H_p0_in_victor)[0] - p0_offset
        p0_in_arm_victor = extract_from_matrix(H_p0_in_arm_victor)[0]
        print("p0_in_victor",p0_in_victor)
        data_dic['p_0_in_victor'].append(p0_in_victor.copy())
        data_dic['p_0_in_arm_victor'].append(p0_in_arm_victor.copy())
        
        # elif state_ == 'calibration':
        if len(data_dic['p_0_in_mocap']) >= 30:
            # store_h5_dict('calibration_data.h5',data_dic)

            R_optimal, t_optimal =calibration(pts=data_dic['p_0_in_arm_mocap'], 
                        pts_transformed=data_dic['p_0_in_arm_victor'])
            data_dic['transformation'] = [R_optimal, t_optimal]
            # store_h5_dict('calibration_data.h5',data_dic)

            print('calibrated_mat H_victor_in_mocap',R_optimal,t_optimal)


        # H_victor_in_mocap = H_arm_in_mocap@ H_arm_victor_in_arm_mocap @  H_victor_in_arm
        # H_mocap_in_victor_root = np.linalg.inv(H_victor_in_mocap)

        # pos,qua = extract_from_matrix(H_victor_in_mocap)
        # print("pos",pos)
        # print("qua",qua)


        # H_chunk_in_mocap_msg = tfwrapper.get_transform(parent="mocap_world",child="mocap_iron_chunk_iron_chunk")
        # H_chunk_in_mocap = build_mat_from_transform(H_chunk_in_mocap_msg)
        # H_chunk_in_victor_root =  H_mocap_in_victor_root @ H_chunk_in_mocap
        # pos,qua = extract_from_matrix(H_chunk_in_victor_root)
        # rivz_pub.rviz_sphere_publish(pos.reshape(1,3),frame_type='base')

        # delta = p_victor_left_finger_b_link3_in_victor_frame-pos

        # print("delta",delta)

    # print('calibrated_mat H_victor_in_mocap',H_victor_in_mocap)    
    # print('pos,qua', extract_from_matrix(H_victor_in_mocap))
    data_dic['transformation'] = [R_optimal, t_optimal]
    # store_h5_dict('calibration_data.h5',data_dic)
    node.destroy_node()
    rclpy.shutdown()



def calibration(pts, pts_transformed):
    # Apply SE(3) transformation
    def apply_transformation(points, R, t):
        return points @ R.T + t

    # Define cost function
    def cost_function(R, t, points1, points2):
        return anp.sum((apply_transformation(points1, R, t) - points2) ** 2)
        

    # Generate data
    points1 = anp.array(pts)
    points2 = anp.array(pts_transformed)

    # Define the Product Manifold SE(3)
    R_manifold = SpecialOrthogonalGroup(3)
    t_manifold = Euclidean(3)
    SE3 = Product([R_manifold, t_manifold])

    # Cost function for pymanopt
    @pymanopt.function.autograd(SE3)
    def cost(R, t):
        return cost_function(R, t, points1, points2)

    # Initial guess (identity transformation: R as identity, t as zero)
    initial_R = anp.eye(3)
    initial_t = anp.zeros(3)
    initial_Rt = (initial_R, initial_t)

    # Problem and optimizer
    problem = pymanopt.Problem(manifold=SE3, cost=cost)
    optimizer = TrustRegions()

    # Run the optimization
    result = optimizer.run(problem, initial_point=initial_Rt)
    Rt_optimal = result.point

    # Extract optimized R and t
    R_optimal, t_optimal = Rt_optimal

    # Transform points1 using the optimized transformation
    points1_transformed = apply_transformation(points1, R_optimal, t_optimal)

    delta = np.linalg.norm(points1_transformed - points2)
    print("Optimization result:", delta)

    return R_optimal, t_optimal
if __name__ == "__main__":
    main()
