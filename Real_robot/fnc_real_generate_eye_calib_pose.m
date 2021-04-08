function qs_measure_reorder = fnc_real_generate_eye_calib_pose(T_obj, robot_last_joint_pose, r, row, column, z_angle, shift_level)
%FNC_REAL_GENERATE_EYE_ Summary of this function goes here
%   Detailed explanation goes here
% robot_with_tool = my_new_dh_robot();
%% Calculate approximate ball position
T_obj = [T_obj;1];
SE3_lj = convert_real_robot_pos(robot_last_joint_pose);
R_ = [-1,0,0;0,1,0;0,0,-1]';
Tool_ = [R_,[0,0,370]';
        zeros(1,3),1];
Ball_pos = SE3_lj *Tool_ * T_obj;
Ball_pos = Ball_pos(1:3);
theta_ball = atan2(-Ball_pos(2),Ball_pos(1));
R_ball = rotz(-theta_ball/pi*180);
T_ball = [R_ball, Ball_pos;zeros(1,3),1];
%% Generate Measuring Pose
[Ts,p_measure] = gen_eye_calibration_pos(T_ball, r, row, column, z_angle, shift_level);
% qs_measure = robot_with_tool.ikine(Ts);
qs_measure = zeros(size(Ts,3),6);
Ts(1:3,4,:) = Ts(1:3,4,:)/1000;
qs_measure(1,:) = exp_ikine(Ts(:,:,1),zeros(6,1),2)';
for i = 2:size(Ts,3)
    qs_measure(i,:) = exp_ikine(Ts(:,:,i),qs_measure(i-1,:)',2)';
end
%% Visualizing to validate
view_measure_pose(Ts,p_measure,1);
qs_measure_reorder = ZigZag(qs_measure,row,column);
qs_measure_reorder = qs_measure_reorder * 180 / pi;
end

