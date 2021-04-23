clear;
clc;
%% Prepare robot
surfix = './experiment/experiment_0422/'; 
number = 5;

filename = strcat(surfix,'Tool_t_qs',num2str(number),'.mat');   
load(filename);
filename = strcat(surfix,'robot_poe_init.mat');   
load(filename)
robot_poe.T_tool = Tool_T;
%% parameters
n_balls = 2;
n_measure_each_ball = 32;
type = 1;
threshold = 1e-11;
%% Prepare measuring pose and measure data
% filename = strcat(surfix,'eye_calib_qs',num2str(number),'.txt');                                    % Change this line to find the right file
filename = strcat(surfix,'end_data_all.txt');
[p_measures, ~] = read_real_measure_data(filename);
% filename = strcat(surfix,'qs',num2str(number),'.txt');       
filename = strcat(surfix,'qs_all.txt');
qs = read_qs(filename);
%% Calibration
iter = 1;
while 1
    tic;
    [error, delta_poe] = multi_ball_kinematic_calibration_poe(robot_poe, qs, p_measures, type, n_balls, n_measure_each_ball);
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    fprintf('Iteration %d \t takes time %.4f,\t error is %.12f \t links update is %.10f \t g_st update is %.10f \n',[iter, time, error, link_update, gst_update]);
    iter = iter + 1;
    if error < threshold || ( iter > 50) || (link_update < 1e-11) 
        break
    end
end