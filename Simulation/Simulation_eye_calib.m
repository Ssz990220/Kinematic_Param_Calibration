clear;
clc;
%% Generate Tool Frame
R_ = [-1,0,0;0,1,0;0,0,-1]';
Tool = [R_,[0,0,370]';
        zeros(1,3),1];
    
robot_with_tool = my_new_dh_robot(Tool);
robot_without_tool = my_new_dh_robot(eye(4));
Object_T = [eye(3),[1300,0,1100]';
            zeros(1,3),1];
[Ts,p_measure] = gen_eye_calibration_pos(Object_T, 0,2, 4,30,10);

%% Add noise
noise_level = 0.1;
p_measure_noise = normrnd(0,noise_level,size(p_measure));
p_measure = p_measure + p_measure_noise;
%% Generate measuring Pose
qs = robot_with_tool.ikine(Ts);

%% Solve
Ts_without_tool= robot_without_tool.fkine(qs).double();
Ts_noise = zeros(size(Ts_without_tool));
Ts_noise_level = 0.5;
Ts_noise(1:3,4,:) = normrnd(0,Ts_noise_level, [3,size(Ts_without_tool,3)]);
Ts_without_tool = Ts_without_tool + Ts_noise;
T_tool = hand_eye_calibration(Ts_without_tool, p_measure, eye(4));

%% Check results
T_tool
Tool