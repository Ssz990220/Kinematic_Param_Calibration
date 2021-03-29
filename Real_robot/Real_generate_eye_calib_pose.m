addpath(genpath('.\'));
%% add experiment data
T_obj = [-46.1258, 18.3733, 13.4902]';
robot_last_joint_pose = [1256.38 -56.14 1456.13 0.49334 -0.00128 0.86983 -0.003605];
%% Generate pose
% T_obj, robot_last_joint_pose, r, row, column, z_angle, shift_level
qs_measure = fnc_real_generate_eye_calib_pose(T_obj, robot_last_joint_pose, 10, 4, 8, 30, 10);
%% Visualizing to validate
robot = my_new_dh_robot();
robot_view_generate_pose(robot, qs_measure/180*pi);
%% Save
save 'qs5.txt' qs_measure -ascii