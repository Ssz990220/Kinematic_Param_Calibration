clear;
clc;
addpath(genpath('..\'));
%%%
%% genenrate POE parameter with noise
clear;
clc;
robot_poe = my_poe_robot(eye(4), false,0,0, false,0,0);
%%
surfix = './experiment/experiment_0423/';
%% Prepare real robot data
filename = strcat(surfix,'qs5.txt');
angle_list = read_qs(filename);
% angle_list = repmat(angle_list, 4,1);
% Ts_true = read_real_robot_pos('./experiment/experiment_0329/hand_eye_calibration_1614/endT_data.txt');
filename = strcat(surfix,'128_1_32_qs5.txt');
[~, Ts_true] = read_real_measure_data(filename);
Ts_true = avg_ts(Ts_true, 32);
% Ts_true = Ts_true(:,:,1:32);

%% Calibration Hyperparameter
n_points = size(Ts_true,3);
threshold = 1e-11;
calibration_done = false;
iter = 1;
type = 2;
%% Calibration
counter = 0;
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
    counter = counter + 1;
    if error < threshold || ( iter > 50) || (link_update < 1e-9) %|| counter == 2
        break
    end
end

%% Save
save ./experiment/experiment_0422/robot_poe_init.mat robot_poe

