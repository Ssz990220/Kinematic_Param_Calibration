clear;
clc;
addpath('./mr');
%%%
%% genenrate POE parameter with noise
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
z_noise = normrnd(0,0.0005,size(z));
q_noise = normrnd(0,50,size(q));
z_fake = z + z_noise;
for i = 1:6
    z_fake(:,i) = z_fake(:,i)/norm(z_fake(:,i));
end
q_fake = q + q_noise;
true_link = [z;q];

g_st0 = [0,0,1,1270;
        0,-1,0,0;
        1,0,0,1570;
        0,0,0,1];
Tool = [eye(3),[0,0,100]';
        zeros(1,3),1];
robot_poe = my_poe_robot(z_fake,q_fake,g_st0,Tool);
%% Prepare real robot
link1 = Link('d',495,'a',175,'alpha',pi/2);
link2 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
link3 = Link('d',0,'a',175,'alpha',pi/2);
link4 = Link('d',960,'a',0,'alpha',-pi/2);
link5 = Link('d',0,'a',0,'alpha',pi/2);
link6 = Link('d',135,'a',0,'alpha',0);

robot = SerialLink([link1, link2, link3, link4, link5, link6],'tool',Tool);

%% Generate measuring pose and measure data
% T0 = [eye(3),[500,0,200]';
%             zeros(1,3),1];
% dis =  50 * ones(6,1);
% direction = [1,0,0]';
% measure_per_point = 1;
% n_points = 7;
% r = 200;
% z_angle_list = [0,5,-5,10,-10,15,-15];
n_points = 64;

%% Calibration
threshold = 1e-11;
calibration_done = false;
iter = 1;
iter_times = 3;
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
        [calibration_done, error, delta_poe] = kinematic_calibration_poe_absolute(robot_poe, angle_list,Ts_true, Ts_nominal, n_points, threshold);

        %% Debug
        delta_poe_kine = zeros(size(robot_poe.links));
        for i = 1:robot_poe.n_dof
            delta_poe_kine(:,i) = delta_poe(6*(i-1)+1:6*i); 
        end
        %% Continue
        robot_poe.update_poe(delta_poe);
        time = toc;
        fprintf('Iteration %d takes time %.4f, error is %.6f \n',[iter, time, error]);
        iter = iter + 1;
    end
end
