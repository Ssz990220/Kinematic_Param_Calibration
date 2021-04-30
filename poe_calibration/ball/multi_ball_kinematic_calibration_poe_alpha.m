function [init_error, delta_poe, robot_poe] = multi_ball_kinematic_calibration_poe_alpha(robot_poe, qs, p_measure, type, n_balls, n_measures_ball)
%KINEMATIC_CALIBRATION_POE Summary of this function goes here
%   Reference: ﻿Kinematic-parameter identification for serial-robot calibration based on POE formula
%               A Self-Calibration Method for Robotic Measurement System Robot
%   This is a hybird method of the two paper mentioned above.
%   Param:
[~, Delta_x, delta_poe]= get_G_X(robot_poe, qs, p_measure, type, n_balls, n_measures_ball);
init_error = norm(Delta_x)/length(Delta_x);
g_d = -norm(delta_poe)^2;
%% find alpha
iter = 100;
a = 0;
b = inf;
alpha = 1;
 while iter
        lhs = init_error+0.8*alpha*g_d;
        exp = f(alpha*delta_poe,robot_poe, qs, p_measure, type, n_balls, n_measures_ball);
        rhs = init_error+0.2*alpha*g_d;
        if(exp<=rhs)
            if(exp>=lhs)
                break;
            else
                a = alpha;
                if isinf(b)
                    alpha = 2*alpha;
                else
                    alpha = (a+b)/2;
                end
            end
        else
            b = alpha;
            alpha = (a+b)/2;
        end
    iter = iter-1;
 end
%% Update
robot_poe.update_poe(delta_poe*alpha, type);
end

function [G, Delta_x, delta_poe] = get_G_X(robot_poe, qs, p_measure, type, n_balls, n_measures_ball)
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
    for m = 1:n_balls
            base_idx = 0;
            for i = 1:n_measures_ball
                for j = i+1 : n_measures_ball
                    Delta_x((m-1)*m_n+ base_idx + j - i) = - norm(x_measure(:,(m-1)*n_measures_ball + i) - x_measure(:,(m-1)*n_measures_ball +j))^2;
        %             Delta_x((m-1)*m_n +base_idx + j - i) = norm(x_true(:,i) - x_true(:,j))- norm(x_measure(:,i) - x_measure(:,j));
                    G(((m-1)*m_n+base_idx + j - i),:) = 2 * (x_measure(:,(m-1)*n_measures_ball +i) - x_measure(:,(m-1)*n_measures_ball +j))'...
                        *(J(:,:,(m-1)*n_measures_ball +i)-J(:,:,(m-1)*n_measures_ball +j));
        %             G(((m-1)*m_n +base_idx + j - i),:) = (x_measure(:,i)-x_measure(:,j))'*(J(:,:,i)-J(:,:,j))/norm(x_measure(:,i)-x_measure(:,j));
                end
                base_idx = base_idx + n_measures_ball - i;
            end
    end
    delta_poe = pinv(G)*Delta_x;
end

function error = f(delta_poe, robot_poe, qs, p_measure, type, n_balls, n_measures_ball)
    robot_poe_temp = robot_poe;
    robot_poe_temp.update_poe(delta_poe, type);
    [~, Delta_x, ~]= get_G_X(robot_poe_temp, qs, p_measure, type, n_balls, n_measures_ball);
    error = norm(Delta_x)/length(Delta_x);
end
