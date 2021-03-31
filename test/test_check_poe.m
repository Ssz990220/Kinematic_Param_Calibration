clear;
clc;
addpath(genpath('..\'));
robot = my_new_dh_robot();
R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe = my_poe_robot(T_tool);
%% Denormalize the axis for poe robot to validate algorithm
for i = 1:robot_poe.n_dof
    robot_poe.links(1:3,i) = robot_poe.links(1:3,i)*i;  
end

%% Validation
pose = rand(1,6);
T1 = robot.fkine(pose).double();
T2 = robot_poe.fkine(pose);
norm(T1-T2)     % if this goes to 0, it works.