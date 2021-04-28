clear;
clc;
%% Prepare robot
global robot_poe
surfix = './experiment/experiment_0423/'; 
number = 1;

filename = strcat(surfix,'Tool_t_qs',num2str(number),'.mat');   
load(filename);
filename ='./experiment/experiment_0422/robot_poe_init.mat';   
load(filename)
% robot_poe = my_poe_robot();
robot_poe.T_tool = Tool_T;
%% Prepare measuring pose and measure data
filename = strcat(surfix,'128_1_32_qs5.txt');                                    % Change this line to find the right file
[p_measures_, Ts] = read_real_measure_data(filename);
p_measures = avg_p(p_measures_, 32);
filename = strcat(surfix,'qs5.txt');       
qs = read_qs(filename);
% qs = repmat(qs, 4, 1);
%% Optimal choice
init_O = get_O_multi_balls(qs, p_measures, 1, 32, 1)
mask = main_optimal_conf_multi_balls(qs, p_measures, 20, 12, 4, 3, 1, 32, 1);
optimized_O = get_O_multi_balls(qs(mask,:),p_measures(:,mask), 1, length(mask), 1)



%% Random pose optimization
init_O = 0;
while init_O < 50
clear;
% Prepare robot
robot = my_new_dh_robot();
R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];

robot_poe = my_poe_robot(T_tool, true, 0.001,0.01, true,0,0.05,false);
% parameters
% For ball %
n_balls = 1;

n_measure_each_ball = 128;
rand_measure_pose = true;
rand_pose = false;
% For measure %
r =10;
z_angle = 45;
shift_level = 10;
threshold = 1e-11;
type = 1;
% Generate cube position
T_balls = gen_ball_pos(n_balls);
Ball_pos = [ 1533.78165281459,-14.4954655756398,1218.23109460511]';
theta_ball = atan2(-Ball_pos(2),Ball_pos(1));
R_ball = rotz(-theta_ball/pi*180);
T_balls = [R_ball, Ball_pos;zeros(1,3),1];
% Generate measuring pose and measure data
%     z_angle = z_angle_list(mod(iter,size(z_angle_list,2))+1);
if rand_measure_pose
    [Ts, p_measures] = gen_ball_measure_pos(T_balls, r,z_angle,shift_level, n_measure_each_ball);
else
    for i = 1:n_balls
        row =16;
        column = 8;
        [Ts(:,:,((i-1)*n_measure_each_ball+1):i*n_measure_each_ball),p_measures(:,((i-1)*n_measure_each_ball+1):i*n_measure_each_ball)] = gen_eye_calibration_pos(T_balls(:,:,i), r, row, column, z_angle, shift_level);
    end
end
% ikine
tic;
qs = zeros(n_balls*n_measure_each_ball, 6);
Ts(1:3,4,:) = Ts(1:3,4,:)/1000;
if rand_pose
    for i = 1:n_balls*n_measure_each_ball
        qss = exp_ikine(Ts(:,:,i),zeros(6,1),1)';
        qss = qss(any(qss,1),:);
        qs(i,:) = qss(randi([1,size(qss,2)]),:);
        if qs(i,:) == zeros(6,1)
            error('ikine result not found');
        end
    end
else
    for i = 1:n_balls*n_measure_each_ball
        if i == 1
            qs(i,:) = exp_ikine(Ts(:,:,i),zeros(6,1),2)';
        else
            qs(i,:) = exp_ikine(Ts(:,:,i),qs(i-1,:)',2)';
        end
    end
end
% qs = robot.ikine(Ts);
time = toc;
fprintf('Inverse kinematics takes %.4f sec to complete\n',time);
% Add_noise
% noise = (rand(size(p_measures)) - 0.5) * 0.01;
% p_measures = p_measures + noise;
% Optimize configuration
init_O = get_O_multi_balls(qs, p_measures, 1, 64, 1)
end
mask = main_optimal_conf_multi_balls(qs, p_measures, 45, 32, 4, 3, 1, 32, 1);
optimized_O = get_O_multi_balls(qs(mask,:),p_measures(:,mask), 1, length(mask), 1)