clear;
clc;
% Specify folder
% surfix = './experiment/experiment_0422/';      % Change this line to match the date and time
surfix = './../../gocator_pcl/src/pcl_pub/results/0423_0.1mm/';
for number = 1:4
% Load data--all in one
filename = strcat(surfix,num2str(number),'.txt');                                    % Change this line to find the right file
[p_measure{number}, Ts{number}] = read_real_measure_data(filename);
end
Ts_avg  = SE3_avg(Ts(:,:,1:8:end));
p_measure_avg = mean(p_measure,2);

% Initial Guess
R_ = [-1,0,0;0,1,0;0,0,-1]';
init = [R_,[0,0,370]';
        zeros(1,3),1];
    
% Solve
Tool_T = hand_eye_calibration(Ts_avg, p_measure_avg,init);

% check error
% Ball_Pos = ball_pos(p_measure, Ts, Tool_T);
% error = std(Ball_Pos')

% Save
Tool_T_path = strcat(surfix, 'Tool_t_qs_avg.mat');                             % Change this line to save the data in the file you want
save(Tool_T_path, 'Tool_T');

