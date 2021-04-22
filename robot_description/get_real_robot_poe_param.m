clear;
clc;
addpath(genpath('..\'));
%%%
%% genenrate POE parameter with noise
clear;
clc;
robot_poe = my_poe_robot(eye(4), false,0,0, false,0,0);
%%
surfix = './experiment/experiment_0408/eye_calibration_1716/';
%% Prepare real robot data
filename = strcat(surfix,'qs_all.txt');
angle_list = read_qs(filename);
% Ts_true = read_real_robot_pos('./experiment/experiment_0329/hand_eye_calibration_1614/endT_data.txt');
filename = strcat(surfix,'end_data_all.txt');
[~, Ts_true] = read_real_measure_data(filename);
%% Calibration Hyperparameter
n_points = size(Ts_true,3);
threshold = 1e-11;
calibration_done = false;
iter = 1;
type = 2;
%% Calibration
while ~calibration_done
    tic;
    Ts_nominal = zeros(size(Ts_true));
    for i = 1:n_points
        Ts_nominal(:,:,i) = robot_poe.fkine(angle_list(i,:));
    end
    [error, delta_poe] = kinematic_calibration_poe_absolute(robot_poe, angle_list,Ts_true, Ts_nominal, n_points, type);
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    fprintf('Iteration %d takes time %.4f, error is %.10f, update is %.10f \n',[iter, time, error, link_update]);
    iter = iter + 1;
    if error < threshold || ( iter > 50) || (link_update < 1e-9) 
        break
    end
%         if update < threshold
%             break
%         end
end

%% Save
save ./experiment/experiment_0422/robot_poe_init.mat robot_poe

