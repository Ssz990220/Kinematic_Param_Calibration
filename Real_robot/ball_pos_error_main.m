clear;
clc;
% Specify folder
% surfix = './experiment/';      % Change this line to match the date and time
surfix = './../../gocator_pcl/src/pcl_pub/results/0423_0.1mm/';
error = [];
for number = 1:4
Tool_T_path = strcat(surfix, 'Tool_t_qs',num2str(1),'.mat');                             % Change this line to save the data in the file you want
load(Tool_T_path, 'Tool_T');

% Load data--all in one
filename = strcat(surfix,num2str(number),'.txt');                                    % Change this line to find the right file
[p_measure, Ts] = read_real_measure_data(filename);

% check error
Ball_Pos = ball_pos(p_measure, Ts, Tool_T);
Tool_T_record{number} = Tool_T;
% error = std(Ball_Pos');
number_sample = size(p_measure,2);
E = 0;
counter = 0;
for i=1:number_sample
    for j=(i+1):number_sample
        E = E + norm(Ball_Pos(:,i)-Ball_Pos(:,j));
        counter = counter+1;
    end
end
E = E/counter;
error = [error;E];

for i = 1:3
    subplot(3,1,i)
    plot(Ball_Pos(i,:))
    hold on
end
end
error