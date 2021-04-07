clear;
clc;
load Ts.mat Ts
n_points = size(Ts,3);
qs = zeros(6,n_points);
R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];

% for i = 1:n_points
%     Ts(1:3,4,i) = Ts(1:3,4,i)/1000;
%     qs(:,i) = exp_ikine(Ts(:,:,i),zeros(6,1),2);
% end

Ts = Ts(:,:,4);
Ts(1:3,4) = Ts(1:3,4)/1000;
qs = exp_ikine(Ts,zeros(6,1),1);
qs = qs';
robot = my_new_dh_robot();
robot_view_generate_pose(robot, qs);