clear;
clc;
%% prepare data
surfix = './experiment/DATA/0428/'; 
filename = strcat(surfix, 'O40.txt');
[p_measures,Ts, qs] = read_all_in_one(filename);
n_measure_each_ball = size(p_measures, 2);
mask = 1:n_measure_each_ball;
%% Parameters
n_balls = 1;
type = 1;
%% prepare init robot
robot_poe = my_poe_robot(eye(4));
% Ts = robot_poe.fkines(qs);
% filename = strcat(surfix,'x.mat');   
% load(filename);
% robot_poe.initialize(x, type);
%% Generated Parameters
% [Ts, p_measure] = avg_ts_p(Ts_raw, p_measure_raw, n_balls * n_measure_each_ball);
%% First POE Absolute Calibration
% robot_poe = get_real_robot_poe_pendant(Ts, qs, robot_poe);
%% First hand-eye Calibration
R_ = [-1,0,0;0,1,0;0,0,-1]';
init = [R_,[0,0,370]';
        zeros(1,3),1];
Tool_T = hand_eye_calibration(Ts, p_measures,init);
% robot_poe.T_tool = Tool_T;
% filename = strcat(surfix,'Tool_t_O40.mat');   
% load(filename);
robot_poe.T_tool = Tool_T;
%% Iterational Update
iter_times = 5;
for i = 1:iter_times
    
    [error, robot_poe] = multi_ball_kinematic_calibration_poe_optim(robot_poe, qs(mask,:), p_measures(:,mask), type, n_balls, length(mask));
    temp_robot = robot_poe;
    temp_robot.T_tool = eye(4);
    Ts = temp_robot.fkines(qs);
    Tool_T = hand_eye_calibration(Ts, p_measures,Tool_T);
    robot_poe.T_tool = Tool_T;
end

%% Save
x = robot_poe.output(type);
error = round(error * 1000);
filename = strcat(surfix, sprintf('x_E0%d_clean.mat',error));
save(filename, 'x');
filename = strcat(surfix, sprintf('Tool_t_E0%d_clean.mat',error));
save(filename, 'Tool_T');