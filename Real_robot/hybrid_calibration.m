clear;
clc;
%% prepare init robot
global robot_poe
robot_poe = my_poe_robot(eye(4));
%% prepare data
surfix = './experiment/experiment_0423/';
filename = strcat(surfix,'qs5.txt');
qs = read_qs(filename);
filename = strcat(surfix,'128_1_32_qs5.txt');
[p_measure_raw, Ts_raw] = read_real_measure_data(filename);

%% Parameters
n_balls = 1;
n_measure_each_ball = 32;
R_ = [-1,0,0;0,1,0;0,0,-1]';
init = [R_,[0,0,370]';
        zeros(1,3),1];
%% Generated Parameters
[Ts, p_measure] = avg_ts_p(Ts_raw, p_measure_raw, n_balls * n_measure_each_ball);
%% First POE Absolute Calibration
get_real_robot_poe_pendant(Ts, qs);
%% First hand-eye Calibration
Tool_T = hand_eye_calibration(Ts, p_measure,init);
robot_poe.T_tool = Tool_T;
%% Iterational Update
iter_times = 3;
for i = 1:iter_times
    fnc_real_POE_ball_calib(p_measure, qs, n_balls, n_measure_each_ball);
    Ts = robot_poe.fkines(qs);
    Tool_T = hand_eye_calibration(Ts, p_measure,init);
    robot_poe.T_tool = Tool_T;
end

%% Save
