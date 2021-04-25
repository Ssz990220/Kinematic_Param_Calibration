clear;
clc;
error = 0;
counter = 0;
surfix = './experiment/experiment_0423/'; 


filename ='./experiment/experiment_0422/robot_poe_init.mat'; 
load(filename)
% robot_poe = my_poe_robot(eye(4));
filename = strcat(surfix,'128_1_32_qs5.txt');        
[~, T_act] = read_real_measure_data(filename);
% T_act = avg_ts(T_act, 32);
T_act = T_act(:,:,1:32);
filename = strcat(surfix,'qs5.txt');       
qs = read_qs(filename);
n_test = size(qs, 1);
T_mea = zeros(4,4,n_test);
for i = 1:n_test
    pose = qs(i,:);
    T_mea(:,:,i) = robot_poe.fkine(pose);
end

for i = 1:n_test
    for j = (i+1):n_test
        counter = counter+1;
        error = error + norm(norm(T_act(1:3,4,i)-T_act(1:3,4,j))-norm(T_mea(1:3,4,i)-T_mea(1:3,4,j)));
    end
end
error_relative = error / counter

counter = 0;
error = 0;
% T_mea = zeros(4,4,n_test);
for i = 1:n_test
    error_local = MatrixLog6(T_act(:,:,i)*TransInv(T_mea(:,:,i)));
    error = error + norm(error_local);
    counter = counter + 1;
end
error_abso = error / counter

%% Original 
clear;
error = 0;
counter = 0;
surfix = './experiment/experiment_0423/'; 

robot_poe = my_poe_robot(eye(4));
filename = strcat(surfix,'128_1_32_qs5.txt');        
[~, T_act] = read_real_measure_data(filename);
% T_act = avg_ts(T_act, 32);
T_act = T_act(:,:,1:32);
filename = strcat(surfix,'qs5.txt');       
qs = read_qs(filename);
n_test = size(qs, 1);
T_mea = zeros(4,4,n_test);
for i = 1:n_test
    pose = qs(i,:);
    T_mea(:,:,i) = robot_poe.fkine(pose);
end

for i = 1:n_test
    for j = (i+1):n_test
        counter = counter+1;
        error = error + norm(norm(T_act(1:3,4,i)-T_act(1:3,4,j))-norm(T_mea(1:3,4,i)-T_mea(1:3,4,j)));
    end
end
error_relative = error / counter

counter = 0;
error = 0;
% T_mea = zeros(4,4,n_test);
for i = 1:n_test
    error_local = MatrixLog6(T_act(:,:,i)*TransInv(T_mea(:,:,i)));
    error = error +norm(error_local);
    counter = counter + 1;
end
error_abso = error / counter