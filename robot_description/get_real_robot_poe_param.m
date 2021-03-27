clear;
clc;
addpath(genpath('..\'));
%%%
%% genenrate POE parameter with noise
clear;
clc;

robot_poe = my_poe_robot(eye(4), false,0,0);
%% Prepare real robot data
file = fopen('.\experiment_0324\2125_hand_eye_calibration\qs5.txt','r');
formatSpec = '%f %f %f %f %f %f %f\n';
qs = fscanf(file,formatSpec,[6, Inf]);
qs = qs';
angle_list = qs(2:28,:);
angle_list(:,5) = -angle_list(:,5);
angle_list = angle_list/180*pi;
fclose(file);
Ts_true = read_real_robot_pos('.\experiment_0324\2125_hand_eye_calibration\endT_data.txt');
%% Calibration Hyperparameter
n_points = 27;
threshold = 1e-11;
calibration_done = false;
iter = 1;
iter_times = 3;
type = 2;
%% Calibration
for t = 1:iter_times
    while ~calibration_done
        tic;
        Ts_nominal = zeros(size(Ts_true));
        for i = 1:n_points
            Ts_nominal(:,:,i) = robot_poe.fkine(angle_list(i,:));
        end
        [calibration_done, error, delta_poe] = kinematic_calibration_poe_absolute(robot_poe, angle_list,Ts_true, Ts_nominal, n_points, threshold, type);
        old_links = robot_poe.links;
        old_gst = robot_poe.g_st_poe;
        robot_poe.update_poe(delta_poe, type);
        time = toc;
        update = max(max(max(abs(old_links - robot_poe.links))), max(old_gst-robot_poe.g_st_poe));
        fprintf('Iteration %d takes time %.4f, error is %.10f, update is %.10f \n',[iter, time, error, update]);
        iter = iter + 1;
        if update < 2.5
            break
        end
    end
end
