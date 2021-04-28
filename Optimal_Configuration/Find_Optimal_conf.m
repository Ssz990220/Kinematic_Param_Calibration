clear;
clc;
while 1
[qs, T_balls, O] = gen_optimal_config(20);
% robot = my_new_dh_robot();
% robot_view_generate_pose(robot, qs,0.1);
%% Save
time = clock;
time = round(time);
time_str = sprintf('%d%d%d',time(4:6));
O = round(O);
O_str = sprintf('%d',O);

surfix = './experiment/experiment_0427/Gen_Pose/'; 
qs_degree = qs / pi * 180;
filename = strcat(surfix,'qs_64_O',O_str,'_',time_str,'.mat');
save(filename, 'qs_degree');
filename = strcat(surfix,'T_balls_O',O_str,'_',time_str,'.mat');
save(filename, 'T_balls');
end