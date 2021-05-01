clear;
clc;
%% Prepare robot
surfix = './experiment/DATA/0428/'; 
number = 1;

filename = strcat(surfix,'Tool_t_O40.mat');   
load(filename);
filename ='./experiment/experiment_0428/robot_poe_init.mat';   
load(filename)
% load robot_poe.mat
% robot_poe = my_poe_robot();
robot_poe.T_tool = Tool_T;
% filename = strcat(surfix,'x.mat');   
% load(filename);
% robot_poe.initialize(x);
%% parameters
n_balls = 1;
type = 1;
threshold = 1e-11;
%% Prepare measuring pose and measure data
filename = strcat(surfix, 'O40.txt');
[p_measures,Ts, qs] = read_all_in_one(filename);
n_measure_each_ball = size(p_measures, 2);
%% Optimal Configuration
% init_O = get_O_multi_balls(qs, p_measures, 1, n_measure_each_ball, 1, robot_poe)
% mask = main_optimal_conf_multi_balls(qs, p_measures, n_measure_each_ball, 45, 4, 3, 1, 64, 1, robot_poe);
mask = 1:n_measure_each_ball;
% optimized_O = get_O_multi_balls(qs(mask,:),p_measures(:,mask), 1, length(mask), 1, robot_poe)
%% Calibration
iter = 1;
% qs = qs + ones(size(qs)) * 1e-2;
while 1
    tic;
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    [error, robot_poe] = multi_ball_kinematic_calibration_poe_lsq(robot_poe, qs(mask,:), p_measures(:,mask), type, n_balls, length(mask));
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    fprintf('Iteration %d \t takes time %.4f,\t error is %.12f \t links update is %.10f \t g_st update is %.10f \n',[iter, time, error, link_update, gst_update]);
    iter = iter + 1;
    if error < threshold || ( iter > 1000) %|| (link_update < 1e-11) || error < 0.023
        break
    end
end

%% Save 
% save robot_poe.mat robot_poe