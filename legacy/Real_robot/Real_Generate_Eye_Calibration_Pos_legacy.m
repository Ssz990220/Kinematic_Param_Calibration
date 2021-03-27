clear;
clc;
addpath(genpath('..\'));
R =eye(3);
Tool = [R,[0,0,370]';
        zeros(1,3),1];
robot_with_tool = my_dh_robot(Tool);
%% Calculate approximate ball position
T_obj = [9.30724 -19.0494 -4.02572]';       % Change this line every time
T_obj = [T_obj;1];

SE3_lj = convert_real_robot_pos([1278.25 -5.73 1415.92 0.47329 -0.13596 0.86878 -0.05224]);
R_ = [-1,0,0;0,1,0;0,0,-1]';
Tool_ = [R_,[0,0,370]';
        zeros(1,3),1];
Ball_pos = SE3_lj *Tool_ * T_obj;
Ball_pos = Ball_pos(1:3);
T_ball = [eye(3), Ball_pos;zeros(1,3),1];

%% Generate Measuring Pose
row = 4;
column = 8;
[Ts,p_measure] = gen_eye_calibration_pos(T_ball, 0, row, column, 15,10);
qs_measure = robot_with_tool.ikine(Ts);
%% Visualizing to validate
view_measure_pose(Ts,p_measure,1);
%% Postprocessing & Save
for i = 1:row*column
    qs_measure(:,2) = -qs_measure(:,2);
    qs_measure(:,3) = -qs_measure(:,3);
    qs_measure(:,5) = -qs_measure(:,5);
end
qs_measure_reorder = ZigZag(qs_measure,row,column);
qs_measure_reorder = qs_measure_reorder * 180 / pi;
% save 'qs_measure_7.txt' qs_measure_reorder -ascii
%% Visualizing to validate
robot_view_generate_pose(robot_with_tool,qs_measure);
% for i = 1:row*column
%     robot_with_tool.plot(qs_measure_reorder(i,:));
%     pause(0.5);
% end