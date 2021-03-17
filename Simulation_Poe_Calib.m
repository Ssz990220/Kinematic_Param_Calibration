clear;
clc;
addpath('./mr');
%%%
% genenrate POE parameter with noise
clear;
clc;
z = [0 0 1;
    0 -1 0;
    0 -1 0;
    1 0 0;
    0 -1 0;
    1 0 0]';
q = [0 0 0;
    175 0 495;
    175 0 1395;
    175 0 1570;
    1135 0 1570;
    1270 0 1570]';
z_noise = normrnd(0,0.01,size(z));
q_noise = normrnd(0,5,size(q));
z_fake = z + z_noise;
q_fake = q + q_noise;
%%%

% Prepare robot
link1 = Link('d',495,'a',175,'alpha',pi/2);
link2 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
link3 = Link('d',0,'a',175,'alpha',pi/2);
link4 = Link('d',960,'a',0,'alpha',-pi/2);
link5 = Link('d',0,'a',0,'alpha',pi/2);
link6 = Link('d',135,'a',0,'alpha',0);
Tool = [eye(3),[0,0,100]';
        zeros(1,3),1];
robot = SerialLink([link1, link2, link3, link4, link5, link6],'tool',Tool);
g_st0 = [0,0,1,1270;
        0,-1,0,0;
        1,0,0,1570;
        0,0,0,1];
robot_poe = my_poe_robot(z,q,g_st0,Tool);

% Generate measuring pose and measure data
T0 = [eye(3),[500,0,200]';
            zeros(1,3),1];
dis =  50 * ones(6,1);
direction = [1,0,0]';
measure_per_point = 1;
n_points = 7;
r = 200;
z_angle_list = [0,5,-5,10,-10,15,-15];
threshold = 1e-7;
calibration_done = false;
iter = 1;
while ~calibration_done
    tic;
    z_angle = z_angle_list(mod(iter,size(z_angle_list,2))+1);
    [Ts, T_holes, p_measures] = gen_poe_cal_pos_sim(T0, direction, dis, measure_per_point, n_points, r, z_angle);
    qs = robot.ikine(Ts);   % n_points by 6
    x_true = zeros(3,n_points);
    for i=1:n_points
        x_true(:,i) = T_holes(1:3,4,i);
    end

    % Calibrating
    [calibration_done, error, delta_poe] = kinematic_calibration_poe(robot_poe, qs, p_measures, x_true, n_points, threshold);
    robot_poe.update_poe(delta_poe);
    time = toc;
    fprintf('Iteration %d takes time %.4f, error is %.6f \n',[iter, time, error]);
    iter = iter + 1;
end
