clear;
clc;
surfix = './experiment/experiment_0426/1448/';
filename = strcat(surfix, 'robot_poe_init.mat');
load(filename);
R_ = [-1,0,0;0,1,0;0,0,-1]';
tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe.T_tool = tool;
while 1
[qs, T_balls, O, p_measures] = gen_optimal_config(20, robot_poe);
% robot = my_new_dh_robot();
% robot_view_generate_pose(robot, qs,0.1);
%% Save
time = clock;
time = round(time);
time_str = sprintf('%d%d%d',time(4:6));
O = round(O);
O_str = sprintf('%d',O);

surfix = './experiment/experiment_0427/Gen_Pose4/'; 
save_qs_Tball(surfix, qs, T_balls, O_str, time_str, p_measures);
end

function save_qs_Tball(surfix, qs, T_balls, O_str, time_str, p_measures)
    qs_degree = qs / pi * 180;
    filename = strcat(surfix,'qs_64_O',O_str,'_',time_str,'.mat');
    save(filename, 'qs_degree');
    filename = strcat(surfix,'T_balls_O',O_str,'_',time_str,'.mat');
    save(filename, 'T_balls');
    filename = strcat(surfix,'P_measures_O',O_str,'_',time_str,'.mat');
    save(filename, 'p_measures');
end