clear;
clc;
%% Generate Tool Frame
% T_y = @(alpha_y)[cos(alpha_y),0,sin(alpha_y),0;
%         0,1,0,0;
%         -sin(alpha_y),0,cos(alpha_y),0;
%         0,0,0,1];
% T_z = @(alpha_z)[cos(alpha_z),-sin(alpha_z),0,0;
%         sin(alpha_z),cos(alpha_z),0,0;
%         0,0,1,0;
%         0,0,0,1];
% R = T_y(20/180*pi) * T_z(45/180*pi);
% R = R(1:3,1:3);
% Tool = [R,[10,10,100]';
%         zeros(1,3),1];
R_ = [-1,0,0;0,1,0;0,0,-1]';
Tool = [R_,[0,0,370]';
        zeros(1,3),1];
    
robot_with_tool = my_new_dh_robot(Tool);
robot_without_tool = my_new_dh_robot(eye(4));
% robot_with_tool.plot(zeros(1,6));
% ball_pos = load('ball_pos.mat');
Object_T = [eye(3),[1300,0,1100]';
            zeros(1,3),1];
[Ts,p_measure] = gen_eye_calibration_pos(Object_T, 0,2, 4,30,10);

%% Add noise
noise_level = 0.1;
p_measure_noise = normrnd(0,noise_level,size(p_measure));
p_measure = p_measure + p_measure_noise;
%% Generate measuring Pose
qs = robot_with_tool.ikine(Ts);
%%
% Generate joint angle for ABB
% qs_new = zeros(size(qs));
% for i = 1:8
%     qs_new((i-1)*2 + 1,:) = qs(i,:);
%     qs_new((i-1)*2 + 2,:) = qs(i+8,:);
% end

%% Solve
Ts_without_tool= robot_without_tool.fkine(qs).double();
Ts_noise = zeros(size(Ts_without_tool));
Ts_noise_level = 0.5;
Ts_noise(1:3,4,:) = normrnd(0,Ts_noise_level, [3,size(Ts_without_tool,3)]);
Ts_without_tool = Ts_without_tool + Ts_noise;
% for i=1:size(qs,1)
%     robot_with_tool.plot(qs(i,:));
%     pause(2);
% end
T_tool = hand_eye_calibration(Ts_without_tool, p_measure, eye(4));

%% Check results
% T_tool = [eye(3),[0,0,100]';zeros(1,3),1];
T_tool
Tool