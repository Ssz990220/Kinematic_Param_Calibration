clear;close all;clc;
surfix = './../../gocator_pcl/src/pcl_pub/results/';
robot = {'TP','raw','POE','hybrid'};
obj = {'car_hor','car_ver','car_var','car_avg','cube'};


for j = 4 % obj
    for i = 1:4 % robot
        subplot(1,4,i)
        real_path = strcat(surfix, '/real.mat');
        load(real_path, 'real');
        if j == 5
            real_path = strcat(surfix, '/real_cube.mat');
            load(real_path, 'real');
        end
        filename = strcat(surfix,obj{j},'/Raw_Pos_',robot{i},'.mat');
        load(filename,'Ball_Pos')
        load([surfix,obj{j},'/err2_',robot{i},'.mat'],'err2')
        [norm1,norm2] = plot_error_1(Ball_Pos,err2,real,[robot{i},'_',obj{j}],1,1);
    end
end

x0=0;
y0=10;
width=1550;
height=400;
set(gcf,'position',[x0,y0,width,height])