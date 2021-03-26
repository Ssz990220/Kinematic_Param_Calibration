clear;
clc;
addpath(genpath('..\'));
robot = my_new_dh_robot(eye(4));
robot_poe = my_poe_robot(eye(4));
%% Denormalize the axis for poe robot to validate algorithm
for i = 1:robot_poe.n_dof
    robot_poe.links(1:3,i) = robot_poe.links(1:3,i)*i;  
end

%% Validation
pose = rand(1,6);
T1 = robot.fkine(pose).double();
T2 = robot_poe.fkine(pose);
norm(T1-T2)     % if this goes to 0, it works.