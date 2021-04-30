clear;close all;
clc;

obj = 'car_ver_measure';
batch_size = 5;
surfix = './../../gocator_pcl/src/pcl_pub/results/0429/';
Tool_T_path = strcat(surfix, 'Tool_T0.mat');
load(Tool_T_path, 'Tool_T');

robot_poe = my_poe_robot(eye(4));
% x_path = strcat(surfix, 'x3.mat');
% load(x_path, 'x');
% robot_poe.initialize(x);

real_path = strcat(surfix,obj, '/real.mat');
load(real_path, 'real');

for number = 1:batch_size
% Load data--all in onech
filename = strcat(surfix,obj,'/',num2str(number),'.txt');                                    % Change this line to find the right file
[qs,~,p_measure] = read_data(filename);

Ts = robot_poe.fkines(qs);

% record data
[Ball_Pos{number},Ts_record{number}] = ball_pos(p_measure, Ts, Tool_T);
[err{number},err2{number},distance_matrix] = check_error(Ball_Pos{number},real);
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

% figure()
% titles = {'x','y','z'};
% for i = 1:3
%     subplot(3,1,i)
%     for number = 1:batch_size
%         plot(Ball_Pos{number}(i,:)-mean(Ball_Pos{number}(i,:))-Pos(i,:)+mean(Pos(i,:)))
% %         title(titles{i})
%         xlabel('numbers');ylabel([titles{i},' error/mm'])
% %         labels{number} = (['sample', num2str(number)]);
%         hold on
%     end
% %     legend(labels)
% end

err2_avg = zeros(3,sample_size);
for i = 1:batch_size
    err2_avg = err2_avg + err2{number};
end
err2_avg = err2_avg/batch_size;

figure()
titles = {'x','y','z'};
for i = 1:3
    subplot(4,1,i)
    for number = 1:batch_size
        plot(abs(err2{number}(i,:)))
        xlabel('numbers');ylabel([titles{i},' error/mm'])
%         labels{number} = (['sample', num2str(number)]);
        hold on
        plot(abs(err2_avg(i,:)),'LineWidth',5)
    end
%     legend(labels)
end
subplot(4,1,4)
for i = 1:batch_size
    plot(sqrt(sum(err2{i}.^2,1)))
    hold on
end
plot(sqrt(sum(err2_avg.^2,1)),'LineWidth',5)
xlabel('numbers');ylabel('norm error/mm')

% [err_avg,~,distance_matrix] = check_error(Pos,real);
% figure()
% err_plot = zeros(1,batch_size);
% for i = 1:batch_size
%     err_plot(1,i) = err{i};
% end
% plot(1:batch_size,err_plot)
% hold on
% plot(1:batch_size,err_avg*ones(1,batch_size))
% legend('batch','avg')
% title('errors')
% xlabel('batch')
% ylabel('error/mm')

[err,err2,distance_matrix] = check_error(Pos(:,[1 4 5 8]),real(:,[1 4 5 8]));
sum(abs(err2),2)/4
filename = strcat(surfix,obj,'/filtered_Pos_POE.mat');
save(filename,'Pos')

