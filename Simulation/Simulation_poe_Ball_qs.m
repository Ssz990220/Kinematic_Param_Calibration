clear;
clc;

errors = [];
n_times = 1;
for i = 1:n_times
%% Prepare robot
surfix = './experiment/experiment_0423/'; 
filename = strcat(surfix,'Tool_t_qs1.mat');   
load(filename);
robot = my_new_dh_robot(Tool_T);
% R_ = [-1,0,0;0,1,0;0,0,-1]';
% T_tool= [R_,[0,0,370]';
%         zeros(1,3),1];

robot_poe = my_poe_robot(Tool_T, true, 0.001,0.01, true,0,0.1,false);
%(T_tool, add_joint_shift, omega_shift_level, q_shift_level, add_base_shift, base_shift_omega, base_shift_q, add_angle_noise, angle_error_level, angle_error_decay)
%% Initial Error
error = 0;
counter = 0;
n_test = 10;
T_act = zeros(4,4,n_test);
T_mea = zeros(4,4,n_test);

for i = 1:n_test
    pose = rand(1,6);
    T_act(:,:,i) = robot.fkine(pose).double();
    T_mea(:,:,i) = robot_poe.fkine(pose);
end

for i = 1:n_test
    for j = (i+1):n_test
        counter = counter+1;
        error = error + norm(norm(T_act(1:3,4,i)-T_act(1:3,4,j))-norm(T_mea(1:3,4,i)-T_mea(1:3,4,j)));
    end
end
error_init = error / counter;


%% parameters
type = 1;
n_iter = 1;
n_balls = 1;
n_measure_each_ball = 32;
threshold = 1e-11;
for Iter = 1:n_iter
%% Generate cube position
Ball_pos = [ 1533.78165281459,-14.4954655756398,1218.23109460511]';
theta_ball = atan2(-Ball_pos(2),Ball_pos(1));
R_ball = rotz(-theta_ball/pi*180);
T_balls = [R_ball, Ball_pos;zeros(1,3),1];
%% measure qs
filename = strcat(surfix,'qs5.txt');       
qs = read_qs(filename);
%% Calculate P_measures
n = size(qs,1);
Ts_end = robot.fkine(qs).double();
p_measures = zeros(3,n);
for m = 1:n
    p_measure_global = T_balls(1:3,4) - Ts_end(1:3,4,m);
    p_measure_tool = Ts_end(1:3,1:3,m)'*p_measure_global;
    p_measures(:,m) = p_measure_tool;% + rand([3,1]) * 0.05;
end

%% Visualization
% view_holes(T_balls,100,true);
% view_measure_pose(Ts_end, p_measures, 100, false);
%% Random offset
% offset = rand([1,6]) * 0.3;
% offset_noise = repmat(offset, 32,1);
% qs = qs + offset_noise;
%% Calibration
iter = 1;
while 1
    tic;
    [error, delta_poe] = multi_ball_kinematic_calibration_poe(robot_poe, qs, p_measures, type, n_balls, n_measure_each_ball);
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    fprintf('Iteration %d \t takes time %.4f,\t error is %.12f \t links update is %.10f \t g_st update is %.10f \n',[iter, time, error, link_update, gst_update]);
    iter = iter + 1;
    if error < threshold || ( iter > 200) || (link_update < 1e-11) 
        break
    end
end
%% Validation
error = 0;
counter = 0;
T_act = zeros(4,4,n_test);

for i = 1:n_test
    pose = rand(1,6);
    T_act(:,:,i) = robot.fkine(pose).double();
    T_mea(:,:,i) = robot_poe.fkine(pose);
end

for i = 1:n_test
    for j = (i+1):n_test
        counter = counter+1;
        error = error + norm(norm(T_act(1:3,4,i)-T_act(1:3,4,j))-norm(T_mea(1:3,4,i)-T_mea(1:3,4,j)));
    end
end
error = error / counter;
fprintf("Initial error is %.2f, after calibration, error is %.10f \n",[error_init, error]);
errors = [errors, error];
%% End
end
end
fprintf("avg error is %.6f",mean(errors));