addpath(genpath('.\'));
%% add experiment data
T_obj = [40.4416, -2.25177, 16.8473]';                              %  Ball-Center position measured by 3-D camera ( in tool-frame)
robot_last_joint_pose = [981.27 631.64 1441.14 0.30059 -0.62179 0.67799 0.25171];       % Spacial posture of the end joint of robot in x-y-z, unit quatinion
%% Generate pose
% parameters are: T_obj, robot_last_joint_pose, r, row, column, z_angle, shift_level
% check fnc_real_generate_eye_calib_pose for detialed info
qs_measure = fnc_real_generate_eye_calib_pose(T_obj, robot_last_joint_pose, 10, 4, 8, 30, 10);
%% Visualizing to validate
robot = my_new_dh_robot();
figure
robot_view_generate_pose(robot, qs_measure/180*pi);
%% Save
save '.\experiment\experiment_0408\eye_calibration_1716\qs7.txt' qs_measure -ascii      % Change this line each time to save to the right file