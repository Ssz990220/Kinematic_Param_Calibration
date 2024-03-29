clear;
clc;
% Specify folder
surfix = './experiment/experiment_0422/';      % Change this line to match the date and time
error = [];
for number = 5:7
Tool_T_path = strcat(surfix, 'Tool_t_qs',num2str(number),'.mat');                             % Change this line to save the data in the file you want
load(Tool_T_path, 'Tool_T');

% Load data--all in one
filename = strcat(surfix,'eye_calib_qs',num2str(number),'.txt');                                    % Change this line to find the right file
[p_measure, Ts] = read_real_measure_data(filename);

% Load qs
filename = strcat(surfix,'qs',num2str(number),'.txt');   
angle_list = read_qs(filename);
% Get POE Ts
Tool_T_path = strcat(surfix, 'robot_poe_init.mat');
load (Tool_T_path,'robot_poe')
Ts = zeros(size(Ts));
for i = 1:size(angle_list,1)
    Ts(:,:,i) = robot_poe.fkine(angle_list(i,:));
end
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
% for i = 1:3
%     subplot(3,1,i)
%     plot(Ball_Pos(i,:))
%     hold on
% end
end
plot(error)