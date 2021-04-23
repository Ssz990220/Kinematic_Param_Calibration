clear;
clc;
% Specify folder
% surfix = './experiment/';      % Change this line to match the date and time
surfix = './../../gocator_pcl/src/pcl_pub/results/0423_0.1mm/';
error = [];
for number = 6
Tool_T_path = strcat(surfix, 'Tool_t_qs',num2str(3),'.mat');                             % Change this line to save the data in the file you want
load(Tool_T_path, 'Tool_T');

% Load data--all in one
filename = strcat(surfix,num2str(number),'.txt');                                    % Change this line to find the right file
[p_measure, Ts] = read_real_measure_data(filename);
number_sample = size(p_measure,2);

% check error
Ball_Pos = ball_pos(p_measure, Ts, Tool_T);
holes_reorder = zeros(3, 8, number_sample/8);
for i = 1:8
    for j = 1:number_sample/8
        holes_reorder(:,i,j) = Ball_Pos(:,i + (j-1)*8);
    end
end

holes_reorder_rm = [];
test = zeros(1,number_sample/8);
Pos = zeros(3,8);
for i = 1:3
    for j = 1:8
        test(1,:) = holes_reorder(i,j,:);
        [A,TF] = rmoutliers(test);
        if any(TF)
            disp(['error in hole ',num2str(j),' at ',num2str(i),': ',num2str(test(TF))])
        end
        Pos(i,j) = mean(A);
    end
end
end