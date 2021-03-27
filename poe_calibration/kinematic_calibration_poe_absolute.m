function [error, delta_poe] = kinematic_calibration_poe_absolute(robot_poe, qs,Ts_true, Ts_nominal,n_points, type)
%KINEMATIC_CALIBRATION_POE Summary of this function goes here
%   Reference: ﻿Kinematic-parameter identification for serial-robot calibration based on POE formula
%               A Self-Calibration Method for Robotic Measurement System Robot
%   This is a hybird method of the two paper mentioned above.
%   Param:
delta_gg6 = zeros([6*n_points,1]);
if type ==1
    Jacob = zeros([6*n_points,robot_poe.n_dof*6]);
elseif type == 2
    Jacob = zeros([6*n_points,robot_poe.n_dof*6 + 6]);
end
for i = 1:n_points
    delta_gg = MatrixLog6(Ts_true(:,:,i)*TransInv(Ts_nominal(:,:,i)));
    delta_gg6((i-1)*6+1:i*6) = se3ToVec(delta_gg);
    j = robot_poe.get_J(qs(i,:), type);
    if type == 1
        Jacob((i-1)*6+1:i*6,:) = j;
    elseif type == 2
        Jacob((i-1)*6+1:i*6,:) = j;
    end
end
delta_poe = pinv(Jacob)*delta_gg6;
% delta_poe = (Jacob'*Jacob)\(Jacob'*delta_gg6);
error = norm(delta_poe);
end
