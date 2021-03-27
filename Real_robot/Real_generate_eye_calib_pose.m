addpath(genpath('.\'));
%% add experiment data
T_obj = [9.30724 -19.0494 -4.02572]';
robot_last_joint_pose = [1278.25 -5.73 1415.92 0.47329 -0.13596 0.86878 -0.05224];
%% Generate pose
% T_obj, robot_last_joint_pose, r, row, column, z_angle, shift_level
qs_measure = fnc_real_generate_eye_calib_pose(T_obj, robot_last_joint_pose, 0, 2, 4, 15, 10);
%% Visualizing to validate
robot = my_new_dh_robot();
robot_view_generate_pose(robot, qs_measure/180*pi);