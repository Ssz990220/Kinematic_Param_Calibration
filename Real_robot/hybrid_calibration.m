clear;
clc;
%% Parameters
n_balls = 1;
type = 1;
Optimized_O = false;
save_all = true;
with_TP = false;
alg = 'lsq';
%% prepare data
surfix = './experiment/DATA/0428/'; 
filename = strcat(surfix, 'O40.txt');
[p_measures,Ts, qs] = read_all_in_one(filename);
n_measure_each_ball = size(p_measures, 2);
%% prepare init robot
robot_poe = my_poe_robot(eye(4));
if ~with_TP
    Ts = robot_poe.fkines(qs);
end
% filename = strcat(surfix,'x.mat');   
% load(filename);
% robot_poe.initialize(x, type);
%% First POE Absolute Calibration
if with_TP
    robot_poe = get_real_robot_poe_pendant(Ts, qs, robot_poe);
end
%% First hand-eye Calibration
tic;
if with_TP
    filename = strcat(surfix,'Tool_t_O40.mat');   
    load(filename);
else
    R_ = [-1,0,0;0,1,0;0,0,-1]';
    init = [R_,[0,0,370]';
            zeros(1,3),1];
    Tool_T = hand_eye_calibration(Ts, p_measures,init);
end
robot_poe.T_tool = Tool_T;
time = toc;
fprintf('Initial T_tool generated, took %.2f seconds to complete \n', time);
%% Saving parameter
surfix = './experiment/DATA/';
date = round(clock);
date = strcat(num2str(date(2),'%02d'),num2str(date(3),'%02d'));
surfix = strcat(surfix, date, '/');
dirname = strcat(surfix, alg,'/');
if with_TP
    TP = 'TP_';
    dirname = strcat(dirname, TP);
else
    TP = '';
end
if Optimized_O
    dirname = strcat(dirname,'Optimized_O/');
    save_path = strcat(dirname,'OO_');
else
    dirname = strcat(dirname,'Non_Optimized_O/');
    save_path = dirname;
end
mkdir(dirname);
%% Init save
if save_all
    x = robot_poe.output(type);
    filename = strcat(save_path,'x_clean0.mat');
    save(filename, 'x');
    filename = strcat(save_path,'Tool_t_clean0.mat');
    save(filename, 'Tool_T');
end

%% Iterational Update
iter_times = 5;
for i = 1:iter_times
    if Optimized_O
        tic;
        init_O = get_O_multi_balls(qs, p_measures, n_balls, n_measure_each_ball, type, robot_poe);
        disp('Finding optimal configurations...');
        mask = main_optimal_conf_multi_balls(qs, p_measures, 45, 32, 4, 4, n_balls, n_measure_each_ball, type, robot_poe);
        optimized_O = get_O_multi_balls(qs(mask,:), p_measures(:,mask), n_balls, length(mask), type, robot_poe);
        time = toc;
        fprintf('Initial O is %.4f, optimized O is %.4f. Takes %.2f to complete.\n',[init_O, optimized_O, time]);
    else
        mask = 1:n_measure_each_ball;
    end
    tic;
    if strcmp(alg, 'lsq')
        [error, robot_poe] = multi_ball_kinematic_calibration_poe_lsq(robot_poe, qs(mask,:), p_measures(:,mask), type, n_balls, length(mask));
    elseif strcmp(alg,'search')
        [error, robot_poe] = multi_ball_kinematic_calibration_poe_optim(robot_poe, qs(mask,:), p_measures(:,mask), type, n_balls, length(mask));
    end
    time = toc;
    fprintf('Iteration %d is done, error is %.4f, takes %.2f to complete... \n',[i, error, time]);
    temp_robot = robot_poe;
    temp_robot.T_tool = eye(4);
    Ts = temp_robot.fkines(qs);
    tic;
    Tool_T = hand_eye_calibration(Ts, p_measures,Tool_T);
    time = toc;
    fprintf('%dth T_tool generated, took %.2f seconds to complete \n', [i,time]);
    robot_poe.T_tool = Tool_T;
    if save_all 
        x = robot_poe.output(type);
        error = round(error * 1000);
        filename = strcat(save_path,TP, sprintf('x_E0%d_clean%d.mat',[error,i]));
        save(filename, 'x');
        filename = strcat(save_path,TP, sprintf('Tool_t_E0%d_clean%d.mat',[error,i]));
        save(filename, 'Tool_T');
        disp(strcat('Save to file ',filename));
    end
    disp('Current Tool_T is');
    disp(Tool_T);
end

%% Save
if ~save_all
    x = robot_poe.output(type);
    error = round(error * 1000);
    filename = strcat(save_path, TP, sprintf('x_E0%d_clean.mat',error));
    save(filename, 'x');
    filename = strcat(save_path, TP, sprintf('Tool_t_E0%d_clean.mat',error));
    save(filename, 'Tool_T');
    disp(strcat('Save to file ',filename));
end