clc;clear;
x0 = [0.0   0.0   -0.863   0 0 0]
starting_error = victor_misalignment_error_fn(x0)


% T_vt_to_vr = eye(4);
% T_vt_to_vr(1:3, 4) = [x0(1); x0(2); x0(3)];
% T_vt_to_vr(1:3, 1:3) = rotx(x0(4)) * roty(x0(5)) * rotz(x0(6));
% left_arm_delta  = T_mocap_to_vt * T_vt_to_vr * T_vr_to_left * gripper_to_tc - T_mocap_to_ts * ts_to_left_tc;
% right_arm_delta = T_mocap_to_vt * T_vt_to_vr * T_vr_to_right * gripper_to_tc - T_mocap_to_ts * ts_to_right_tc;


[x_final, final_error] = fminsearch(@(x) victor_misalignment_error_fn(x), x0)
p_final_vt_to_vr = x_final(1:3)
q_final_vt_to_vr = rot2quat(rotx(x_final(4)) * roty(x_final(5)) * rotz(x_final(6)))'
T_final_vt_to_vr = trans(p_final_vt_to_vr);
T_final_vt_to_vr(1:3, 1:3) = quat2rot(q_final_vt_to_vr)

function [error_sq] = victor_misalignment_error_fn(x)
% seq: 0
% stamp: 
%   secs: 1505015351
%   nsecs: 948163748
% frame_id: mocap_world
% mocap_VictorTorso_VictorTorso
% Position (xyz):     1.39499147201   -0.21303664215   0.190467192458
% Quaternion (xyzw):  0.00181548594902   -0.0685656860051   0.0055026682105   0.997629776699
p_mocap_to_vt = [1.39499147201   -0.21303664215   0.190467192458]';
q_mocap_to_vt = [0.00181548594902   -0.0685656860051   0.0055026682105   0.997629776699]';
T_mocap_to_vt = trans(p_mocap_to_vt);
T_mocap_to_vt(1:3, 1:3) = quat2rot(q_mocap_to_vt);

% seq: 0
% stamp: 
%   secs: 1505015404
%   nsecs: 738838196
% frame_id: mocap_world
% table_surface
% Position (xyz):     2.60724062873   -0.122182237342   0.410364260567
% Quaternion (xyzw):  0.0040894898356   -0.0659802483303   0.0180640558812   0.997649022847
p_mocap_to_ts = [2.60724062873   -0.122182237342   0.410364260567];
q_mocap_to_ts = [0.0040894898356   -0.0659802483303   0.0180640558812   0.997649022847];
T_mocap_to_ts = trans(p_mocap_to_ts);
T_mocap_to_ts(1:3, 1:3) = quat2rot(q_mocap_to_ts);


% seq: 0
% stamp: 
%   secs: 1505015446
%   nsecs:  31879902
% frame_id: victor_root
% victor_left_gripper_palm_surface
% Position (xyz):     0.80250024386   0.649807251124   0.994679072826
% Quaternion (xyzw):  -0.0382252603757   0.884463203956   -0.384756465545   0.261201325684
p_vr_to_left = [0.80250024386   0.649807251124   0.994679072826]';
q_vr_to_left = [-0.0382252603757   0.884463203956   -0.384756465545   0.261201325684]';
T_vr_to_left = trans(p_vr_to_left);
T_vr_to_left(1:3, 1:3) = quat2rot(q_vr_to_left);

% seq: 0
% stamp: 
%   secs: 1505015462
%   nsecs: 131983042
% frame_id: victor_root
% victor_right_gripper_palm_surface
% Position (xyz):     0.808505717947   -0.555776934324   1.0106142289
% Quaternion (xyzw):  0.637685514275   0.680074952922   0.357476944324   -0.0553667550833
p_vr_to_right = [0.808505717947   -0.555776934324   1.0106142289];
q_vr_to_right = [0.637685514275   0.680074952922   0.357476944324   -0.0553667550833];
T_vr_to_right = trans(p_vr_to_right);
T_vr_to_right(1:3, 1:3) = quat2rot(q_vr_to_right);

gripper_to_table_corner = [0 0 0.117475 1]';
ts_to_left_table_corner = [-0.381 0.5334 0 1]';
ts_to_right_table_corner = [-0.381 -0.5334 0 1]';


T_vt_to_vr = eye(4);
T_vt_to_vr(1:3, 4) = [x(1); x(2); x(3)];
T_vt_to_vr(1:3, 1:3) = rotx(x(4)) * roty(x(5)) * rotz(x(6));
left_arm_delta  = T_mocap_to_vt * T_vt_to_vr * T_vr_to_left * gripper_to_table_corner - T_mocap_to_ts * ts_to_left_table_corner;
right_arm_delta = T_mocap_to_vt * T_vt_to_vr * T_vr_to_right * gripper_to_table_corner - T_mocap_to_ts * ts_to_right_table_corner;


error_sq = sum(left_arm_delta.^2 + right_arm_delta.^2);

end