function [Ts,p_measure] = gen_eye_calibration_sim(T0, r, n_row, n_column, z_angle,shift_level)
%GEN_EYE_CALIBRATION_POS
%   Generate camera position for eye calibration.
%   T0 is SE3 for the ball in world frame.
%   r is the camera visual distance
%   Ry, Rz is two angle.
%   n_pose tells the program how many poses to generate
%     y_range =15;
%     z_range = 120;
    angle_y_idx = 0:1:n_row-1;
    angle_y = (45+(60-45)/(n_row-1)*angle_y_idx)/180*pi;
    angle_z_idx = 0:1:n_column-1;
    angle_z = (-z_angle+z_angle*2/(n_column-1)*angle_z_idx)/180*pi;
    T_y = @(alpha_y)[cos(alpha_y),0,sin(alpha_y),0;
        0,1,0,0;
        -sin(alpha_y),0,cos(alpha_y),0;
        0,0,0,1];
    T_z = @(alpha_z)[cos(alpha_z),-sin(alpha_z),0,0;
            sin(alpha_z),cos(alpha_z),0,0;
            0,0,1,0;
            0,0,0,1];
    T_x = [eye(3),[-r,0,0]';
            zeros(1,3),1];
    T_add = [[0,0,1;0,-1,0;1,0,0]',[0,0,0]';zeros(1,3),1];
    Ts = zeros(4,4,n_column*n_row);
    p_measure = zeros(3,n_column*n_row);
    for i=1:n_row
        rotation_y = T_y(angle_y(i));
        for j = 1:n_column
            rotation_z = T_z(angle_z(j));
            pos_shift = (rand([3,1]) - 0.5)*2*shift_level;
%             pos_shift = [1,1,1]';
            T_tran = [eye(3),pos_shift;zeros(1,3),1];
            Ts(:,:,(i-1)*n_column + j)=T0*rotation_y*rotation_z*T_x*T_add*T_tran;
            p_measure(:,(i-1)*n_column + j) = [0,0,r]'- pos_shift;
        end
    end
end

