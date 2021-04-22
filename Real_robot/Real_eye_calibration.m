clear;
clc;
%% Specify folder
surfix = './experiment/';      % Change this line to match the date and time

for number = 1
%% Load data--all in one
filename = strcat(surfix,'eye_calib_qs (',num2str(number),').txt');                                    % Change this line to find the right file
[p_measure, Ts] = read_real_measure_data(filename);

%% Initial Guess
R_ = [-1,0,0;0,1,0;0,0,-1]';
init = [R_,[0,0,370]';
        zeros(1,3),1];
    
%% Solve
Tool_T = hand_eye_calibration(Ts, p_measure,init);

%% check error
% Ball_Pos = ball_pos(p_measure, Ts, Tool_T);
% error = std(Ball_Pos')

%% Save
Tool_T_path = strcat(surfix, 'Tool_t_qs',num2str(number),'.mat');                             % Change this line to save the data in the file you want
save(Tool_T_path, 'Tool_T');
end
