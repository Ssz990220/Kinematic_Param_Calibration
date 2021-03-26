
% R_ = [1,0,0;0,-1,0;0,0,-1]';
R_ = [-1,0,0;0,1,0;0,0,-1];
% R_ = eye(3);
Tool_ = [R_,[0,0,370]';
        zeros(1,3),1];
addpath(genpath('..\'));
robot = my_new_dh_robot(Tool_);
robot.plot([0 0 0 0 0 0]);
% robot.plot(zeros(6));
file = fopen('.\experiment_0324\2125_hand_eye_calibration\qs5.txt','r');
formatSpec = '%f %f %f %f %f %f\n';
qs = fscanf(file,formatSpec,[6, Inf]);
qs = qs/180*pi;
qs(5,:) = -qs(5,:);
% robot_view_generate_pose(robot, qs');
Ts_end = robot.fkine(qs').double();
point_end = Ts_end(1:3,4,:);