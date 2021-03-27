clear;
clc;
%% Prepare robot
robot = my_new_dh_robot();

R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe = my_poe_robot(T_tool, true, 0.5,50,false);

%% parameters
T0 = [eye(3),[500,0,200]';
            zeros(1,3),1];
dis =  50 * ones(6,1);
direction = [1,0,0]';
measure_per_point = 1;
n_points = 7;
r = 200;
z_angle_list = [0,5,-5,10,-10,15,-15];
threshold = 1e-6;
calibration_done = false;
% n_times = 3;
iter = 1;
type = 1;

%% Generate measuring pose and measure data
%     z_angle = z_angle_list(mod(iter,size(z_angle_list,2))+1);
z_angle  = z_angle_list;
[Ts, T_holes, p_measures] = gen_poe_cal_pos(T0, direction, dis, measure_per_point, n_points, r, z_angle,10);
qs = robot.ikine(Ts);   % n_points by 6
x_true = zeros(3,n_points);
for i=1:n_points
    x_true(:,i) = T_holes(1:3,4,i);
end
%% Calibration

while ~calibration_done
    tic;
    [error, delta_poe] = kinematic_calibration_poe(robot_poe, qs, p_measures, x_true, n_points, type);
    if error < threshold
        break
    end
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    fprintf('Iteration %d takes time %.4f, error is %.6f \n',[iter, time, error]);
    iter = iter + 1;
end
