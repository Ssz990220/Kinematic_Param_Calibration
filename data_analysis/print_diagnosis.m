clear;close all;clc;
surfix = './../../gocator_pcl/src/pcl_pub/results/';
robot = {'raw','TP','POE','hybrid'};
obj = {'car_hor','car_ver','car_var','car_avg','cube'};

ftable = fopen([surfix,'result.txt'],'w');
mask = [1 2 3 4];
fprintf(ftable,'\t\t\t');
for i = mask
    fprintf(ftable,[robot{i},'\t\t\t\t']);
end
fprintf(ftable,'\n');

for j = 1:5 % obj
    fprintf(ftable,[obj{j},'\t\t']);
    for i = mask % robot
        real_path = strcat(surfix, '/real.mat');
        load(real_path, 'real');
        if j == 5
            real_path = strcat(surfix, '/real_cube.mat');
            load(real_path, 'real');
        end
        filename = strcat(surfix,obj{j},'/Raw_Pos_',robot{i},'.mat');
        load(filename,'Ball_Pos')
        load([surfix,obj{j},'/err2_',robot{i},'.mat'],'err2')
        [norm1,norm2] = plot_error_1(Ball_Pos,err2,real,[robot{i},obj{j}],0);
        fprintf(ftable,'%f\t\t',norm2);
    end
    fprintf(ftable,'\n');
end
fclose(ftable);