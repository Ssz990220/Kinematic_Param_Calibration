function [error, delta_poe] = kinematic_calibration_poe(robot_poe, qs, p_measure, x_true, type)
%KINEMATIC_CALIBRATION_POE Summary of this function goes here
%   Reference: ﻿Kinematic-parameter identification for serial-robot calibration based on POE formula
%               A Self-Calibration Method for Robotic Measurement System Robot
%   This is a hybird method of the two paper mentioned above.
%   Param:
n_points = size(qs,1);
x_measure = zeros([3,n_points]);
if type == 2
    J = zeros(3,6*robot_poe.n_dof + 6,n_points);
elseif type == 1
    J = zeros(3,6*robot_poe.n_dof,n_points);
end
for i = 1:n_points
    T = robot_poe.fkine(qs(i,:));
    x_coor4 = T*[p_measure(:,i);1];
    x_measure(:,i) = x_coor4(1:3);
    J_full = robot_poe.get_J(qs(i,:), type);
    J(:,:,i) = [-skew(x_measure(:,i)),eye(3)]*J_full;
end
Delta_x = zeros(n_points*(n_points-1)/2,1);
if type == 2
    G = zeros(n_points*(n_points-1)/2,6*robot_poe.n_dof + 6);
elseif type == 1
    G = zeros(n_points*(n_points-1)/2,6*robot_poe.n_dof);
end
base_idx = 0;
for i = 1:n_points
    for j = i+1 : n_points
        Delta_x(base_idx + j - i) = - norm(x_measure(:,i) - x_measure(:,j))^2 + norm(x_true(:,i)-x_true(:,j))^2;
%         Delta_x(base_idx + j - i) = norm(x_true(:,i) - x_true(:,j))- norm(x_measure(:,i) - x_measure(:,j));
        G((base_idx + j - i),:) = 2 * (x_measure(:,i) - x_measure(:,j))'*(J(:,:,i)-J(:,:,j));
%         G((base_idx + j - i),:) = (x_measure(:,i)-x_measure(:,j))'*(J(:,:,i)-J(:,:,j))/norm(x_measure(:,i)-x_measure(:,j));
    end
    base_idx = base_idx + n_points - i;
end
delta_poe = pinv(G)*Delta_x;
error = norm(delta_poe);
end

