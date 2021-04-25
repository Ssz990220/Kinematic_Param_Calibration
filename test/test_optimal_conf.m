clear;
clc;
%% Prepare robot
global robot_poe
surfix = './experiment/experiment_0423/'; 
number = 1;

filename = strcat(surfix,'Tool_t_qs',num2str(number),'.mat');   
load(filename);
filename ='./experiment/experiment_0422/robot_poe_init.mat';   
load(filename)
% robot_poe = my_poe_robot();
robot_poe.T_tool = Tool_T;
%% Prepare measuring pose and measure data
filename = strcat(surfix,'128_1_32_qs5.txt');                                    % Change this line to find the right file
[p_measures, Ts] = read_real_measure_data(filename);
% p_measures = avg_p(p_measures_, 32);
filename = strcat(surfix,'qs5.txt');       
qs = read_qs(filename);
qs = repmat(qs, 4, 1);
%% Optimal choice
init_O = get_O_multi_balls(qs, p_measures, 1, 128, 1)
mask = main_optimal_conf_multi_balls(qs, p_measures, 20, 12, 4, 3, 1, 32, 1);
optimized_O = get_O_multi_balls(qs(mask,:),p_measures(:,mask), 1, length(mask), 1)