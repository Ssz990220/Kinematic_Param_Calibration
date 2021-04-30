clear;
clc;
% Specify folder
surfix = './experiment/DATA/0428/';     % Change this line to match the date and time
error = [];
batch_size = 3;
for number = 1:1
% Load data--all in one
% filename = strcat(surfix,'eye_calib_qs (',num2str(number),').txt');                                    % Change this line to find the right file
% [p_measure, Ts] = read_real_measure_data(filename);
Tool_T_path = './experiment/DATA/0428/Tool_t_E0290_clean.mat';                             % Change this line to save the data in the file you want
load(Tool_T_path, 'Tool_T');
filename = strcat(surfix,'O40.txt');
% p_measure = read_p_measure(filename);
% filename = strcat(surfix, 'ts_qs1.txt');
% [qs, Ts] = read_ts_qs(filename);
[p_measure,Ts, qs] = read_all_in_one(filename);
% filename = './experiment/experiment_0422/qs6.txt';
% qs = read_qs(filename);
% filename = './experiment/experiment_0422/eye_calib_qs6.txt';
% [p_measure,Ts] = read_real_measure_data(filename);
%% Load POE
robot_poe = my_poe_robot(eye(4));
filename = './experiment/DATA/0428/x_E0290_clean.mat';
load(filename);
robot_poe.initialize(x, 1);
% load robot_poe.mat
robot_poe.T_tool = eye(4);
% Ts = robot_poe.fkines(qs);
%% check error
Ball_Pos{number} = ball_pos(p_measure, Ts, Tool_T);
Tool_T_record{number} = Tool_T;
% error = std(Ball_Pos');
number_sample = size(p_measure,2);
E = 0;
counter = 0;
for i=1:number_sample
    for j=(i+1):number_sample
        E = E + norm(Ball_Pos{number}(:,i)-Ball_Pos{number}(:,j));
        counter = counter+1;
    end
end
E = E/counter;
error = [error;E];
end
error
% titles = {'x','y','z'};
% for i = 1:3
%     subplot(3,1,i)
%     for number = 1:batch_size
%         plot(Ball_Pos{number}(i,:)-mean(Ball_Pos{number}(i,:)))
% %         title(titles{i})
%         xlabel('numbers');ylabel([titles{i},' error/mm'])
% %         labels{number} = (['sample', num2str(number)]);
%         hold on
%     end
% end