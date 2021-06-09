function [Ball_pos,Ts] = ball_pos(p_obj, T_robot,Tool)
%FNC_REAL_GENERATE_EYE_ Summary of this function goes here
%   Parameters:     T_obj: Ball position in tool frame measured by structure light camera
%                          robot_last_joint_pose: Spacial posture of the end joint of robot in x-y-z, unit quatinion
%                          r: distance between center of the ball to the center of the tool frame
%                          row: number of rows of generated measuring pose, check gen_eye_calibration_pos for more info
%                          column: number of columns of generated measuring pose, check gen_eye_calibration_pos for more info
%                          z_angle: check gen_eye_calibration_pos for more info
%                          shift_level: random shift distance to avoid singularity
%% Calculate approximate ball position
sample_size = size(p_obj,2);
Ball_pos = zeros(3,sample_size);
Ts = zeros(4,4,sample_size);
for i = 1:sample_size
    p_obji = [p_obj(:,i);1];
    Ball_posi = T_robot(:,:,i) * Tool * p_obji;
    Ball_pos(:,i) = Ball_posi(1:3);
    Ts(:,:,i) = T_robot(:,:,i) * Tool;
end