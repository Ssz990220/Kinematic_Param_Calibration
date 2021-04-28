function [Ts,p_measure, qs, Fail] = gen_ball_measure_pos_collision_check(T0, r, z_angle,shift_level, n_measures_ball,rand_pose)
%GEN_EYE_CALIBRATION_POS
%   Generate camera position for eye calibration.
%   T0 is SE3 for the ball in world frame.
%   r is the camera visual distance
%   Ry, Rz is two angle.
%   n_pose tells the program how many poses to generate
%     y_range =15;
%     z_range = 120;
    n_balls = size(T0,3);
    T_y = @(alpha_y)[cos(alpha_y),0,sin(alpha_y),0;
        0,1,0,0;
        -sin(alpha_y),0,cos(alpha_y),0;
        0,0,0,1];
    T_z = @(alpha_z)[cos(alpha_z),-sin(alpha_z),0,0;
            sin(alpha_z),cos(alpha_z),0,0;
            0,0,1,0;
            0,0,0,1];
    T_x = [eye(3),[-r,0,0]';                                                    % shift on x for r, to pull the camera away from the ball a little bit
            zeros(1,3),1];
%     T_add = [[0,0,1;0,-1,0;1,0,0]',[0,0,0]';zeros(1,3),1];
    T_add = [[0,0,1;0,1,0;-1,0,0]',[0,0,0]';zeros(1,3),1];
    Ts = zeros(4,4,n_measures_ball*n_balls);
    qs = zeros(n_balls*n_measures_ball, 6);
    p_measure = zeros(3,n_measures_ball*n_balls);
    Fail = false;
    for n = 1:n_balls
        ball_pos = T0(1:3,4,n)';
        fail = 0;
        for i=1:n_measures_ball
            while 1
                angle_y = (-45+90* rand())/180*pi;
                angle_z = (rand() - 0.5)*z_angle*2/180*pi;
                rotation_y = T_y(angle_y);
                rotation_z = T_z(angle_z);
                pos_shift = (rand([3,1]) - 0.5)*2*shift_level;          % add a random position shift to avoid singularity (result from simulation)
                T_tran = [eye(3),pos_shift;zeros(1,3),1];
                T =T0(:,:,n)*rotation_y*rotation_z*T_x*T_add*T_tran;
                T_meter = T;
                T_meter(1:3,4) = T_meter(1:3,4)/1000;
                if rand_pose
                    qss = exp_ikine(T_meter,zeros(6,1),1)';
                    qss = qss(any(qss,1),:);
                    q = qss(randi([1,size(qss,2)]),:);
                    if q == zeros(6,1)
                        error('ikine result not found');
                    end
                else
                    if i == 1
                        q = exp_ikine(T_meter,zeros(6,1),2)';
                    else
                        q = exp_ikine(T_meter,qs(i-1 + (n-1) * n_measures_ball,:)',2)';
                    end
                end
                close all;
                if ~detect_between_robot_environment(q/pi*180,0,ball_pos)                
                    Ts(:,:,(n-1)*n_measures_ball + i) = T;
                    qs(i + (n-1) * n_measures_ball,:) = q;
                    p_measure(:,(n-1)*n_measures_ball + i) = [0,0,r]'+ pos_shift;
                    break;
                else
                    fail = fail + 1;
                    if fail > 2000
                        fprintf('Failed!\n');
                        Fail = true;
                        break
                    end
                end
            end
            if Fail
                break
            end
        end
    end
    p_measure = -p_measure;
end

