clear;
clc;
%% Prepare robot
robot = my_new_dh_robot();

R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe = my_poe_robot(T_tool, true, 0.2,1, false,0,0,false);

%% parameters
n_holes = 16;
T_holes = gen_random_hole_pos(n_holes);
measure_per_point = 1;
r = 50;
z_angle = 45;
threshold = 1e-11;
iter = 1;
type = 1;

%% Generate measuring pose and measure data
%     z_angle = z_angle_list(mod(iter,size(z_angle_list,2))+1);
[Ts, p_measures] = gen_poe_cal_pos(T_holes, measure_per_point, r, z_angle,10);
% view_holes(T_holes,10,true);
% view_measure_pose(Ts, p_measures, 10, false);
qs = robot.ikine(Ts);   % n_points by 6
x_true = zeros(3,n_holes);
for i=1:n_holes
    x_true(:,i) = T_holes(1:3,4,i);
end
%% Calibration

while 1
    tic;
    [error, delta_poe] = kinematic_calibration_poe(robot_poe, qs, p_measures, x_true, type);
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    fprintf('Iteration %d\t takes time %.4f, \t error is %.12f \n',[iter, time, error]);
    iter = iter + 1;
    if error < threshold
        break
    end
    if iter > 30
        break
    end
end
