clear;close all;
clc;

obj = 'cube_measure';
experiment_number = 5;
batch_size = 1;
surfix = './../../gocator_pcl/src/pcl_pub/results/0429/';
Tool_T_path = strcat(surfix, 'Tool_T.mat');
load(Tool_T_path, 'Tool_T');

robot_poe = my_poe_robot(eye(4));
x_path = strcat(surfix, 'x.mat');
load(x_path, 'x');
robot_poe.initialize(x);

for number = 1:batch_size
surfix = './../../gocator_pcl/src/pcl_pub/results/0429/';
% Load data--all in one
filename = strcat(surfix,obj,'/',num2str(number),'.txt');                                    % Change this line to find the right file
[qs,~,p_measure] = read_data(filename);

Ts = robot_poe.fkines(qs);

% record data
[Ball_Pos{number},Ts_record{number}] = ball_pos(p_measure, Ts, Tool_T);
% error = std(Ball_Pos');
end

filename = strcat(surfix,obj,'/Raw_Pos_POE.mat');
save(filename,'Ball_Pos')
save([surfix,obj,'/Raw_Ts_POE.mat'],'Ts_record')