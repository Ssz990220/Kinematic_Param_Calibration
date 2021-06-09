function [error, delta_poe] = multi_kinematic_calibration_poe(robot_poe, qs, p_measure, x_true, type, n_holes_cube, n_cube, n_measures)
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

Delta_x = zeros(n_holes_cube*(n_holes_cube-1)*n_cube/2,1);
if type == 1
    G = zeros(n_holes_cube*(n_holes_cube-1)*n_cube/2,6*robot_poe.n_dof);
elseif type == 2
    G = zeros(n_holes_cube*(n_holes_cube-1)*n_cube/2,6*robot_poe.n_dof + 6);
elseif type == 3
    G = zeros(n_holes_cube*(n_holes_cube-1)*n_cube/2,7*robot_poe.n_dof);
end
m_n = n_holes_cube*(n_holes_cube-1)/2*n_measures;
m_t = n_holes_cube*(n_holes_cube-1)/2;
for m = 1:n_cube
    for t = 1:n_measures
        base_idx = 0;
        for i = 1:n_holes_cube
            for j = i+1 : n_holes_cube
                Delta_x((m-1)*m_n +(t-1) * m_t + base_idx + j - i) = - norm(x_measure(:,(m-1)*n_holes_cube + i) - x_measure(:,(m-1 + (t-1)*n_cube)*n_holes_cube +j))^2 + norm(x_true(:,(m-1)*n_holes_cube +i)-x_true(:,(m-1 + (t-1)*n_cube)*n_holes_cube +j))^2;
    %             Delta_x((m-1)*m_n +base_idx + j - i) = norm(x_true(:,i) - x_true(:,j))- norm(x_measure(:,i) - x_measure(:,j));
                G(((m-1)*m_n +(t-1) * m_t+base_idx + j - i),:) = 2 * (x_measure(:,(m-1)*n_holes_cube +i) - x_measure(:,(m-1 + (t-1)*n_cube)*n_holes_cube +j))'*(J(:,:,(m-1)*n_holes_cube +i)-J(:,:,(m-1+ (t-1)*n_cube)*n_holes_cube +j));
    %             G(((m-1)*m_n +base_idx + j - i),:) = (x_measure(:,i)-x_measure(:,j))'*(J(:,:,i)-J(:,:,j))/norm(x_measure(:,i)-x_measure(:,j));
            end
            base_idx = base_idx + n_holes_cube - i;
        end
    end
end
delta_poe = pinv(G)*Delta_x;
error = norm(delta_poe);
end



function lambda = AG()

end

function G = get_G(n_cube, n_holes_cube, n_measures, G, m_n, x_measure, J)
    for m = 1:n_cube
        for t = 1:n_measures
            base_idx = 0;
            for i = 1:n_holes_cube
                for j = i+1 : n_holes_cube
                    G(((m-1)*m_n +(t-1) * m_t+base_idx + j - i),:) = 2 * (x_measure(:,(m-1)*n_holes_cube +i) - x_measure(:,(m-1 + (t-1)*n_cube)*n_holes_cube +j))'*(J(:,:,(m-1)*n_holes_cube +i)-J(:,:,(m-1+ (t-1)*n_cube)*n_holes_cube +j));
                end
            end
            base_idx = base_idx + n_holes_cube - i;
        end
    end
end


function Delta_x = get_Delta_x(n_cube, n_holes_cube, n_measures, Delta_x, m_n, x_measure, x_true)
    for m = 1:n_cube
        for t = 1:n_measures
            base_idx = 0;
            for i = 1:n_holes_cube
                for j = i+1 : n_holes_cube
                    Delta_x((m-1)*m_n +(t-1) * m_t + base_idx + j - i) = - norm(x_measure(:,(m-1)*n_holes_cube + i) - x_measure(:,(m-1 + (t-1)*n_cube)*n_holes_cube +j))^2 + norm(x_true(:,(m-1)*n_holes_cube +i)-x_true(:,(m-1 + (t-1)*n_cube)*n_holes_cube +j))^2;
                end
            end
            base_idx = base_idx + n_holes_cube - i;
        end
    end
end