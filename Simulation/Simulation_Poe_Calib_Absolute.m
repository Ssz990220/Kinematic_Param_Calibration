clear;
clc;
addpath(genpath('..\'));
%%%
%% genenrate POE parameter with noise
clear;
clc;
Tool = [eye(3),[0,0,100]';
        zeros(1,3),1];
robot_poe = my_poe_robot(Tool, true, 0.0005,50);
%% Prepare real robot
robot = my_new_dh_robot(Tool);
%% Calibration Hyperparameter
n_points = 64;
threshold = 1e-11;
calibration_done = false;
iter = 1;
iter_times = 3;
type = 1;

%% Calibration
for t = 1:iter_times
    angle_list = 2*(rand([n_points,6])-0.5)*pi;
    Ts_true = robot.fkine(angle_list).double();
    while ~calibration_done
        tic;
        Ts_nominal = zeros(size(Ts_true));
        for i = 1:n_points
            Ts_nominal(:,:,i) = robot_poe.fkine(angle_list(i,:));
        end
        norm(mean(Ts_true - Ts_nominal,3));
        [calibration_done, error, delta_poe] = kinematic_calibration_poe_absolute(robot_poe, angle_list,Ts_true, Ts_nominal, n_points, threshold, type);

        %% Debug
%         delta_poe_kine = zeros(size(robot_poe.links));
%         for i = 1:robot_poe.n_dof
%             delta_poe_kine(:,i) = delta_poe(6*(i-1)+1:6*i); 
%         end
        %% Continue
        robot_poe.update_poe(delta_poe, type);
        time = toc;
        fprintf('Iteration %d takes time %.4f, error is %.6f \n',[iter, time, error]);
        iter = iter + 1;
    end
end
