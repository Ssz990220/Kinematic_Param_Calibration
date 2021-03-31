function [Ts, p_measures] = gen_poe_cal_pos(T_holes, measure_per_point, r, z_angle, shift_level)
%GEN_POE_CAL_POS Summary of this function goes here
%   We assume the calibration bar is placed on the plane and directed to positive x
%   axis. The approximate distance between each two holes are pre-known.
%
%   Param: measure_per_point is the number of measurement on one hole. (To
%   eliminate random noise in 3D sterao camera.)
%   Param: n_points indicates how many holes is on the line.
%   Param: direction, the direction that the bar is directing.
%   Param: distance, array (n_points-1,1), approximate distance between
%   two neighbouring holes.
%   Param: r, double, camera range.
%
%   Return: Ts, n_points*measure_pre_point by SE3. The spacial position for
%   each measurement.
%     if measure_per_point ~= 1
%         angle_z_idx = 0:1:measure_per_point-1;
%         angle_z = (-60+120/(measure_per_point-1)*angle_z_idx)/180*pi;
%     else
%         angle_z = z_angle/180*pi;
%     end
    n_points = size(T_holes, 3);
    angle_y = randi([45,70],1,n_points)/180*pi;
    angle_z = randi([-z_angle,z_angle],1,n_points)/180*pi;
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
    T_add = [[0,0,1;0,1,0;-1,0,0]',[0,0,0]';zeros(1,3),1];
    Ts = zeros(4,4,n_points * measure_per_point);
    p_measures = zeros(3,n_points * measure_per_point);
    for i = 1:n_points
        T_current_hole = T_holes(:,:,i);
        for j = 1:measure_per_point
            rotation_z = T_z(angle_z(i));
            rotation_y = T_y(angle_y(i));
            pos_shift = (rand([3,1]) - 0.5)*2*shift_level;
            T_tran = [eye(3),pos_shift;zeros(1,3),1];
            Ts(:,:,(i-1)*measure_per_point + j) = T_current_hole*rotation_y*rotation_z*T_x*T_add*T_tran;
            p_measures(:,(i-1)*measure_per_point + j) = [0,0,r]'+ pos_shift;
        end
    end
    p_measures = - p_measures;
end

