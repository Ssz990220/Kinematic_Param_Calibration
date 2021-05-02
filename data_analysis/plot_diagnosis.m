clear;close all;clc;
surfix = './../../gocator_pcl/src/pcl_pub/results/';
robot = {'raw','TP','POE','hybrid'};
obj = {'car_hor','car_ver','car_var','car_avg','cube'};

for j = [3 4] % obj
    for i = 1:4 % robot
        real_path = strcat(surfix, '/real.mat');
        load(real_path, 'real');
        if j == 5
            real_path = strcat(surfix, '/real_cube.mat');
            load(real_path, 'real');
        end
        filename = strcat(surfix,obj{j},'/Raw_Pos_',robot{i},'.mat');
        load(filename,'Ball_Pos')
        load([surfix,obj{j},'/err2_',robot{i},'.mat'],'err2')
        [norm1,norm2] = plot_error(Ball_Pos,err2,real,[robot{i},obj{j}],1);
    end
end