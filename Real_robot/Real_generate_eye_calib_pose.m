addpath(genpath('.\'));
%% add experiment data
T_obj =[-20.3944, 26.9389, -4.06452]';                              %  Ball-Center position measured by 3-D camera ( in tool-frame)
robot_last_joint_pose = [1306.53,152.515,1474.83,0.368703,0.0970936,0.892093,-0.242487];       % Spacial posture of the end joint of robot in x-y-z, unit quatinion
%% Generate pose
% parameters are: T_obj, robot_last_joint_pose, r, row, column, z_angle, shift_level
% check fnc_real_generate_eye_calib_pose for detialed info
[qs_measure, ball_pos] = fnc_real_generate_eye_calib_pose(T_obj, robot_last_joint_pose, 10, 8, 8, 30, 10);
%% Visualizing to validate
robot = my_new_dh_robot();
% figure
% robot_view_generate_pose(robot, qs_measure/180*pi);
%% Collision Detection
close all;
figure
for i = 1:64
    if collision_check(qs_measure(i,:),0,ball_pos', 0.01)
        fprintf('Q %d collision\n',i);
    end
end
%% Save
save '.\experiment\experiment_0430\qs8.txt' qs_measure -ascii      % Change this line each time to save to the right file