clear;
clc;
%% Load data
surfix = './experiment/experiment_0427/Gen_Pose/';
filename = strcat(surfix, 'qs_64_O80_13294.mat');
load(filename);
filename = strcat(surfix, 'T_balls_O80_13294.mat');
load(filename);
qs = qs_degree/180*pi;
robot = my_new_dh_robot();
%% Parameters
% mask = 1:32;
%% Generate p_measures
p_measures = inverse_p_measures(T_balls, qs);
% filename = strcat(surfix, 'P_measures_O79_215913.mat');
% load(filename);
%% Initial T_measures
T_measures_init = robot.fkine(qs).double();
%% Poe robot
load './experiment/experiment_0426/1448/robot_poe_init.mat';
% robot_poe = my_poe_robot();
R_ = [-1,0,0;0,1,0;0,0,-1]';
tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe.T_tool = tool;
%% Initial_O
init_O = get_O_multi_balls(qs, p_measures, 1, 64, 1, robot_poe);
fprintf('init_O is %.4f\n',init_O);
%% Visualize initial measure posture
% view_holes(T_balls, 10,true);
% view_measure_pose(T_measures_init(:,:,mask), p_measures(:,mask),10, false);
%% new_p_measures
p_measures_new = p_measures * 0.2;
T_measures_new = T_measures_init;
for i = 1:size(qs, 1)
    R = T_measures_init(1:3,1:3,i);
    p_measures_global = R * p_measures_new(:,i);
    T_measures_new(1:3,4,i) = T_balls(1:3,4) - p_measures_global;
end
% view_holes(T_balls, true);
% view_measure_pose(T_measures_new(:,:,mask), p_measures_new(:,mask),0.1, false);

%% new_qs
qs_new = zeros(size(qs));
T_meter = T_measures_new;
T_meter(1:3,4,:) = T_meter(1:3,4,:)/1000;
for i = 1:size(qs,1)
    qs_new(i,:) = exp_ikine(T_meter(:,:,i),qs(i,:)',2)';
end
% robot_view_generate_pose(robot, qs,0.5);

%% In sight check
for i = 1:size(qs,1)
    check = collision_check(qs_new(i,:)/pi*180, 0, T_balls(1:3,4)');
    if check
        fprintf('q %d is not in sight\n',i);
    end
end
%% Double_check
% global robot_poe

init_O = get_O_multi_balls(qs_new, p_measures_new, 1, 64, 1, robot_poe);
fprintf('fixed O is %.4f\n',init_O);