clear;
clc;
%% Prepare robot
surfix = './experiment/experiment_0423/'; 
number = 1;

filename = strcat(surfix,'Tool_t_qs',num2str(number),'.mat');   
load(filename);
filename ='./experiment/experiment_0422/robot_poe_init.mat';   
load(filename)
% robot_poe = my_poe_robot();
robot_poe.T_tool = Tool_T;
%% parameters
n_balls = 1;
n_measure_each_ball = 32;
type = 1;
threshold = 1e-11;
%% Prepare measuring pose and measure data
filename = strcat(surfix,'128_1_32_qs5.txt');                                    % Change this line to find the right file
[p_measures_, Ts] = read_real_measure_data(filename);
% p_measures = p_measures(:,1:32);
p_measures = avg_p(p_measures_, 32);
% p_measures = p_measures_(:,1:32);
filename = strcat(surfix,'qs5.txt');       
qs = read_qs(filename);
% qs = repmat(qs,4,1);
%% Check Pose
% robot = my_new_dh_robot(eye(4));
% qs_ = robot.ikine(Ts(:,:,1:32));
% robot_view_generate_pose(robot, qs,0.1);

%% Visualization (Debug)
Ball_pos = [ 1533.78165281459,-14.4954655756398,1218.23109460511]';
theta_ball = atan2(-Ball_pos(2),Ball_pos(1));
R_ball = rotz(-theta_ball/pi*180);
T_balls = [R_ball, Ball_pos;zeros(1,3),1];
% view_holes(T_balls,10,true);
Ts_end = robot_poe.fkines(qs);
% view_measure_pose(Ts_end, p_measures, 1, false);
%% Calibration
iter = 1;
% qs = qs + ones(size(qs)) * 1e-2;
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
    if error < threshold || ( iter > 400) || (link_update < 1e-11) 
        break
    end
end