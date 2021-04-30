% clear;
% clc;
%% Specify folder
surfix = './experiment/experiment_0426/1448/';      % Change this line to match the date and time

for number = 1
%% Load data--all in one
% filename = strcat(surfix,num2str(number),'.txt');                                    % Change this line to find the right file
% filename = strcat(surfix,'128_1_32_qs5.txt');
% [p_measure, Ts] = read_real_measure_data(filename);
filename = strcat(surfix,'p1.txt');
p_measure = read_p_measure(filename);
% p_measure = p_measure(:,1:32);
filename = strcat(surfix,'ts_qs1.txt');
[qs, Ts] = read_ts_qs(filename);
    
% Ts = Ts(:,:,1:32);
%% Initial Guess
R_ = [-1,0,0;0,1,0;0,0,-1]';
init = [R_,[0,0,370]';
        zeros(1,3),1];

%% Average to cancel noise
% [Ts, p_measure] = avg_ts_p(Ts, p_measure, 32);
%% Solve
Tool_T = hand_eye_calibration(Ts, p_measure,init);

%% check error
% Ball_Pos = ball_pos(p_measure, Ts, Tool_T);
% error = std(Ball_Pos')

%% Save
Tool_T_path = strcat(surfix, 'Tool_t_qs',num2str(number),'.mat');                             % Change this line to save the data in the file you want
save(Tool_T_path, 'Tool_T');
end
