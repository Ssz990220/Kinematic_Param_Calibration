clear;
clc;
%%%
%% genenrate POE parameter with noise
clear;
clc;
Tool = [eye(3),[0,0,100]';
        zeros(1,3),1];
robot_poe = my_poe_robot(Tool, true, 0.001, 0.05, false, 0.05,20, true,0.001,2);
%% Prepare real robot
robot = my_new_dh_robot(Tool);
%% Calibration Hyperparameter
n_points = 64;
threshold = 1e-11;
calibration_done = false;
iter = 1;
iter_times = 1;
type = 3;

%% Initial Error
error = 0;
n_test = 100;
for i = 1:n_test
    pose = rand(1,6);
    T1 = robot.fkine(pose).double();
    T2 = robot_poe.fkine(pose);
    error_local = MatrixLog6(T1*TransInv(T2));
    error = error + norm(error_local);
end
error_init = error / n_test;
%% Calibration
for t = 1:iter_times
    angle_list = 2*(rand([n_points,6])-0.5)*pi;
    Ts_true = robot.fkine(angle_list).double();
%     for i = 1:n_points
%         Ts_true(1:3,4,i) = Ts_true(1:3,4,i) + normrnd(0,0.03,1,3)';
%     end
    while ~calibration_done
        tic;
        Ts_nominal = zeros(size(Ts_true));
        for i = 1:n_points
            Ts_nominal(:,:,i) = robot_poe.fkine(angle_list(i,:));
        end
%         norm(mean(Ts_true - Ts_nominal,3));
        [error, delta_poe] = kinematic_calibration_poe_absolute(robot_poe, angle_list,Ts_true, Ts_nominal, n_points, type);
        old_links = robot_poe.links;
        old_gst = robot_poe.g_st_poe;
        robot_poe.update_poe(delta_poe, type);
        time = toc;
        link_update = max(max(abs(old_links - robot_poe.links)));
        gst_update = max(abs(old_gst - robot_poe.g_st_poe));
        fprintf('Iteration %d \t takes time %.4f,\t error is %.12f \t links update is %.10f \t g_st update is %.10f \n',[iter, time, error, link_update, gst_update]);
        iter = iter + 1;
        if error < threshold
            break
        end
        if iter > 50
            break
        end
    end
end


%% Validation
error = 0;
n_test = 100;
for i = 1:n_test
    pose = rand(1,6);
    T1 = robot.fkine(pose).double();
    T2 = robot_poe.fkine(pose);
    error_local = MatrixLog6(T1*TransInv(T2));
    error = error + norm(error_local);
end
error = error / n_test;

fprintf("Initial error is %.2f, after calibration, error is %.10f \n",[error_init, error]);