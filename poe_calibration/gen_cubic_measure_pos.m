function [Ts, p_measures] = gen_cubic_measure_pos(cubes, r, z_angle_level, shift_level)
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
    Ts = [];
    p_measures = [];
    for i = 1:cubes.n_cubes
        cube = cubes.cubes(i);
        n_points = cube.n_holes;
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
        Ts_ = zeros(4,4,n_points);
        p_measures_ = zeros(3,n_points);

        for f = 1:3     % for each surface
            if cube.type == 1
                angle_y = (rand(1,cube.n_holes_each_face) * 25 + 5)/ 180 * pi;
            elseif cube.type == 2
                angle_y = -(rand(1,cube.n_holes_each_face) * 25 + 5)/ 180 * pi;
            end
            if f == 1
                angle_z =(rand(1,cube.n_holes_each_face) - 0.5) * 2 * z_angle_level / 180 * pi;
                T_current_holes = cube.Ts_face1;
            elseif f == 2
                angle_z =- rand(1,cube.n_holes_each_face) * z_angle_level / 180 * pi;
                T_current_holes = cube.Ts_face2;
            elseif f == 3
                angle_z =rand(1,cube.n_holes_each_face) * z_angle_level / 180 * pi;
                T_current_holes = cube.Ts_face3;
            end
            for h = 1:cube.n_holes_each_line^2
                T_current_hole = T_current_holes(:,:,h);
                rotation_z = T_z(angle_z(h));
                rotation_y = T_y(angle_y(h));
                pos_shift = (rand([3,1]) - 0.5)*2*shift_level;
                T_tran = [eye(3),pos_shift;zeros(1,3),1];
                Ts_(:,:,(f-1)*cube.n_holes_each_face + h) = T_current_hole*rotation_y*rotation_z*T_x*T_add*T_tran;
                p_measures_(:,(f-1)*cube.n_holes_each_face + h) = [0,0,r]'+ pos_shift;
            end
        end
        p_measures_ = - p_measures_;
        Ts = cat(3, Ts, Ts_);
        p_measures = cat(2, p_measures, p_measures_);
    end
end

