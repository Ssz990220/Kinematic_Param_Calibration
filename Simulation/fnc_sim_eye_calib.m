function error = fnc_sim_eye_calib(r, n_row,  n_column, z_angle, shift_level, measure_noise_level, Ts_noise_level)
%FNC_SIM_EYE_CALIB Summary of this function goes here
%   Detailed explanation goes here
%% Generate Tool Frame
R_ = [-1,0,0;0,1,0;0,0,-1]';
Tool = [R_,[0,0,370]';
        zeros(1,3),1];
    
robot_with_tool = my_new_dh_robot(Tool);
robot_without_tool = my_new_dh_robot(eye(4));
Object_T = [eye(3),[1300,0,1100]';
            zeros(1,3),1];
[Ts,p_measure] = gen_eye_calibration_pos(Object_T, r,n_row, n_column,z_angle,shift_level);

%% Add noise
noise_level = measure_noise_level;
p_measure_noise = normrnd(0,noise_level,size(p_measure));
p_measure = p_measure + p_measure_noise;
%% Generate measuring Pose
qs = robot_with_tool.ikine(Ts);

%% Add noise in ee location
Ts_without_tool= robot_without_tool.fkine(qs).double();
Ts_noise = zeros(size(Ts_without_tool));
Ts_noise(1:3,4,:) = normrnd(0,Ts_noise_level, [3,size(Ts_without_tool,3)]);
Ts_without_tool = Ts_without_tool + Ts_noise;

%% Solve

T_tool = hand_eye_calibration(Ts_without_tool, p_measure, eye(4),'off');

%% return results
error = norm(T_tool - Tool);
end

