clear;
clc;
% Specify folder
% surfix = './experiment/';      % Change this line to match the date and time
surfix = './../../gocator_pcl/src/pcl_pub/results/0428/';
experiment_number = 5;
batch_size = 1;
for number = 1:batch_size
Tool_T_path = strcat(surfix, 'target_ball_qs',num2str(experiment_number),'/Tool_t_qs',num2str(1),'.mat');                             % Change this line to save the data in the file you want
load(Tool_T_path, 'Tool_T');

% Load data--all in one
filename = strcat(surfix,'target_ball_qs',num2str(experiment_number),'/',num2str(number),'.txt');                                    % Change this line to find the right file
[p_measure, Ts] = read_real_measure_data(filename);

% record data
[Ball_Pos{number},Ts_record{number}] = ball_pos(p_measure, Ts, Tool_T);
Tool_T_record{number} = Tool_T;
% error = std(Ball_Pos');
end

filename = strcat(surfix,'target_ball_qs',num2str(experiment_number),'/Raw_Pos.mat');
save(filename,'Ball_Pos')
save([surfix,'target_ball_qs',num2str(experiment_number),'/Raw_Ts.mat'],'Ts_record')

error = [];
for number = 1:batch_size
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

figure()
titles = {'x','y','z'};
for i = 1:3
    subplot(3,1,i)
    for number = 1:batch_size
        plot(Ball_Pos{number}(i,:)-mean(Ball_Pos{number}(i,:)))
%         title(titles{i})
        xlabel('numbers');ylabel([titles{i},' error/mm'])
%         labels{number} = (['sample', num2str(number)]);
        hold on
    end
end
% legend(labels)