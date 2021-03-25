function [calibration_done, error, delta_poe] = kinematic_calibration_poe(robot_poe, qs, p_measure, x_true, n_points, thershold)
%KINEMATIC_CALIBRATION_POE Summary of this function goes here
%   Reference: ﻿Kinematic-parameter identification for serial-robot calibration based on POE formula
%               A Self-Calibration Method for Robotic Measurement System Robot
%   This is a hybird method of the two paper mentioned above.
%   Param:
x_measure = zeros(3,n_points);
J = zeros(3,6*robot_poe.n_dof,n_points);
for i = 1:n_points
    T = robot_poe.fkine(qs(i,:));
    x_coor4 = T*[p_measure(:,i);1];
    x_measure(:,i) = x_coor4(1:3);
    J_full = robot_poe.get_J(qs(i,:));
    J(:,:,i) = J_full(4:6,:);
end
Delta_x = zeros(n_points*(n_points-1)/2,1);
G = zeros(n_points*(n_points-1)/2,6*robot_poe.n_dof);
base_idx = 0;
for i = 1:n_points
    for j = i+1 : n_points
        delta_x = - norm(x_measure(:,i) - x_measure(:,j))^2 + norm(x_true(:,i)-x_true(:,j))^2;
        Delta_x(base_idx + j - i) = delta_x;
        G((base_idx + j - i),:) = 2 * (x_measure(:,i) - x_measure(:,j))'*(J(:,:,i)-J(:,:,j));
    end
    base_idx = base_idx + n_points - i;
end
delta_poe = pinv(G)*Delta_x;
% delta_poe = (G'*G)\(G'*Delta_x);
error = norm(delta_poe);
if error<thershold
    calibration_done = true;
else
    calibration_done = false;
end

end

