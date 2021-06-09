function [error, robot_poe] = multi_ball_kinematic_calibration_poe(robot_poe, qs, p_measure, type, n_balls, n_measures_ball)
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
elseif type == 3
    J = zeros(3,7*robot_poe.n_dof,n_points);
end
for i = 1:n_points
    T = robot_poe.fkine(qs(i,:));
    x_coor4 = T*[p_measure(:,i);1];
    x_measure(:,i) = x_coor4(1:3);
    J_full = robot_poe.get_J(qs(i,:), type);
    J(:,:,i) = [-skew(x_measure(:,i)),eye(3)]*J_full;
end

Delta_x = zeros(n_measures_ball*(n_measures_ball-1)*n_balls/2,1);
if type == 1
    G = zeros(n_measures_ball*(n_measures_ball-1)*n_balls/2,6*robot_poe.n_dof);
elseif type == 2
    G = zeros(n_measures_ball*(n_measures_ball-1)*n_balls/2,6*robot_poe.n_dof + 6);
elseif type == 3
    G = zeros(n_measures_ball*(n_measures_ball-1)*n_balls/2,7*robot_poe.n_dof);
end
m_n = n_measures_ball*(n_measures_ball-1)/2;
counter = 1;
for m = 1:n_balls
%         base_idx = 0;
        for i = 1:n_measures_ball
            for j = i+1 : n_measures_ball
%                 Delta_x(counter) = - norm(x_measure(:,(m-1)*n_measures_ball + i) - x_measure(:,(m-1)*n_measures_ball +j))^2;
                Delta_x(counter) = - norm(x_measure(:,i) - x_measure(:,j));
%                 G(counter,:) = 2 * (x_measure(:,(m-1)*n_measures_ball +i) - x_measure(:,(m-1)*n_measures_ball +j))'...
%                     *(J(:,:,(m-1)*n_measures_ball +i)-J(:,:,(m-1)*n_measures_ball +j));
                G(counter,:) = (x_measure(:,i)-x_measure(:,j))'*(J(:,:,i)-J(:,:,j))/norm(x_measure(:,i)-x_measure(:,j));
            counter = counter + 1;
            end
%             base_idx = base_idx + n_measures_ball - i;
        end
end
delta_poe = pinv(G)*Delta_x;
error = mean(abs(Delta_x));
robot_poe.update_poe(delta_poe*0.01, type);
end