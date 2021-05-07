clear;close all;
clc;
% Specify folder
% surfix = './experiment/';      % Change this line to match the date and time
surfix = './../../gocator_pcl/src/pcl_pub/results/0428/';
obj = 'cube_scan';
experiment_number = 5;
batch_size = 1;
Tool_T_path = strcat(surfix, 'target_ball_qs',num2str(experiment_number),'/Tool_t_qs',num2str(1),'.mat');                             % Change this line to save the data in the file you want
load(Tool_T_path, 'Tool_T');
% Tool_T(1:3,4) = [0;0;370];

surfix = './../../gocator_pcl/src/pcl_pub/results/0429/';
for number = 1:batch_size
surfix = './../../gocator_pcl/src/pcl_pub/results/0429/';
% Load data--all in one
filename = strcat(surfix,obj,'/',num2str(number),'.txt');                                    % Change this line to find the right file
[qs,Ts,p_measure] = read_data(filename);

% record data
[Ball_Pos{number},Ts_record{number}] = ball_pos(p_measure, Ts, Tool_T);
end

filename = strcat(surfix,obj,'/Raw_Pos_TP.mat');
save(filename,'Ball_Pos')
save([surfix,obj,'/Raw_Ts_TP.mat'],'Ts_record')