clear;close all;
clc;

obj = 'cube_measure';
experiment_number = 5;
batch_size = 8;
surfix = './../../gocator_pcl/src/pcl_pub/results/0429/';
Tool_T_path = strcat(surfix, 'Tool_T3.mat');
load(Tool_T_path, 'Tool_T');

robot_poe = my_poe_robot(eye(4));
x_path = strcat(surfix, 'x3.mat');
load(x_path, 'x');
robot_poe.initialize(x);

real_path = strcat(surfix,obj, '/real.mat');
load(real_path, 'real');

for number = 1:batch_size
surfix = './../../gocator_pcl/src/pcl_pub/results/0429/';
% Load data--all in one
filename = strcat(surfix,obj,'/',num2str(number),'.txt');                                    % Change this line to find the right file
[qs,~,p_measure] = read_data(filename);

Ts = robot_poe.fkines(qs);

% record data
[Ball_Pos{number},Ts_record{number}] = ball_pos(p_measure, Ts, Tool_T);
[err{number},err2{number},distance_matrix] = check_error(Ball_Pos{number},real);
% error = std(Ball_Pos');
end
sample_size = size(err2{1},2);

filename = strcat(surfix,obj,'/Raw_Pos_POE.mat');
save(filename,'Ball_Pos')
save([surfix,obj,'/Raw_Ts_POE.mat'],'Ts_record')

Pos = zeros(size(Ball_Pos{1}));
test = zeros(1,batch_size);
for k = 1:size(Pos,2)
    error_count = 0;
    for j = 1:3
        for i = 1:batch_size
            test(1,i) = Ball_Pos{i}(j,k);
        end
        [A,TF] = rmoutliers(test);
        if any(TF)
%             disp(['batch ',num2str(i),' outliers at dim ',num2str(j),' on number ',num2str(k)])
%             disp(test(TF))
            error_count = error_count+length(test(TF));
        end
        Pos(j,k) = mean(A);
    end
    disp(['error percentage for number ' num2str(k),': ', num2str(error_count/3/batch_size)])
end

figure()
titles = {'x','y','z'};
for i = 1:3
    subplot(3,1,i)
    for number = 1:batch_size
        plot(Ball_Pos{number}(i,:)-mean(Ball_Pos{number}(i,:))-Pos(i,:)+mean(Pos(i,:)))
%         title(titles{i})
        xlabel('numbers');ylabel([titles{i},' error/mm'])
%         labels{number} = (['sample', num2str(number)]);
        hold on
    end
%     legend(labels)
end

[err,err2,distance_matrix] = check_error(Pos(:,1:8),real(:,1:8));
sum(abs(err2),2)/8

filename = strcat(surfix,obj,'/filtered_Pos_POE.mat');
save(filename,'Pos')

