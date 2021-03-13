function Ts = gen_eye_calibration_pos(T0, r, n_pose)
%GEN_EYE_CALIBRATION_POS 此处显示有关此函数的摘要
%   Generate camera position for eye calibration.
%   T0 is SE3 for the ball in world space.
%   r is the camera visual distance
%   Ry, Rz is two angle.
%   n_pose tells the program how many poses to generate
angle_y = 60/180*pi;
angle_z = 60*2/180*pi/n_pose;
T_y = [

end

