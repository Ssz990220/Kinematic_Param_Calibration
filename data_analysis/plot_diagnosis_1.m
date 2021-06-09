clear;close all;clc;
surfix = './../../gocator_pcl/src/pcl_pub/results/';
robot = {'raw','TP','POE','hybrid'};
obj = {'car_hor','car_ver','car_var','car_avg','cube'};

subplot(1,4,1)
for j = 1 % obj
    for i = 1 % robot
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

subplot(1,4,2)
for j = 1 % obj
    for i = 2 % robot
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

subplot(1,4,3)
for j = 3 % obj
    for i = 1 % robot
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

subplot(1,4,4)
for j = 3 % obj
    for i = 2 % robot
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