clear;
clc;
%% Specify folder
surfix = './experiment/experiment_0408/eye_calibration_1716/';
%% load measure position
% filename = strcat(surfix,'/ball_measure.txt');
% p_measure = read_real_ball_measure(filename);
        
%% Load Last Joint Posture
% filename = strcat(surfix,'endT_data.txt');
% Ts = read_real_robot_pos(filename);

%% Load data--all in one
filename = strcat(surfix,'eye_calib_qs6.txt');
[p_measure, Ts] = read_real_measure_data(filename);
%% Filter Ts & p_measure input

% Ts_valid_index = [18,19,20,21,22,23,26,27,28,29,30,31,2,3,4,5,6,7,10,11,12,13,14,15];
% Ts_valid = Ts(:,:,Ts_valid_index);
% n_points_used = 26;
% Ts = Ts(:,:,1:n_points_used);
% p_measure = p_measure(:,1:n_points_used);
R_ = [-1,0,0;0,1,0;0,0,-1]';
init = [R_,[0,0,370]';
        zeros(1,3),1];
    
%% Solve
Tool_T = hand_eye_calibration(Ts, p_measure,init)
%% Save
Tool_T_path = strcat(surfix, 'Tool_t_qs7.mat');
save(Tool_T_path, 'Tool_T');
