clear;close all;clc;
surfix = './../../gocator_pcl/src/pcl_pub/results/';
robot = {'raw','TP','POE','hybrid','072','0229'};
obj = {'car','car_ver','car_var','car_var_avg'};
batch_size = [15,5,6,4];

real_path = strcat(surfix, '/real.mat');
load(real_path, 'real');

for j = 1:4 % obj
    for i = 1:6 % robot
        filename = strcat(surfix,obj{j},'/Raw_Pos_',robot{i},'.mat');
        load(filename,'Ball_Pos')
        load([surfix,obj{j},'/err2_',robot{i},'.mat'],'err2')
        [norm1,norm2] = plot_error(Ball_Pos,err2,real,[robot{i},obj{j}],1);
    end
end