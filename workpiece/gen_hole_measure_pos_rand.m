function qs = gen_hole_measure_pos_rand(holes, n_measures)
%GEN_HOLE_MEASURE_POS_RAND Summary of this function goes here
%   Detailed explanation goes here
r = 0;
n_holes = size(holes,2);
qs = zeros(n_measures * n_holes + 1, 6);
figure
hold on
axis equal
for i = 1:n_holes*n_measures
    T_hole = eye(4);
    index = mod(i, n_holes);
    if index == 0
        index = 8;
    end
    T_hole(1:3,4) = holes(:,index);
    view_holes(T_hole,10, false);
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
    T_add = [[0,0,1;0,1,0;-1,0,0]',[0,0,0]';zeros(1,3),1];
    while 1
        angle_y = rand() * 25 + 70;
        angle_z = (rand() - 0.5) * 60;
        rotation_y = T_y(angle_y/180*pi);
        rotation_z = T_z(angle_z/180*pi);
        T_measure=T_hole*rotation_y*rotation_z*T_x*T_add;
        T_meter = T_measure;
        T_meter(1:3,4) = T_meter(1:3,4)/1000;
        q= exp_ikine(T_meter, qs(i,:)');
        p_measure= -[0,0,r]';
        if q(4)>-120 || q(6)>-100
            view_measure_pose(T_measure, p_measure, 10, false);
            qs(i+1,:) = q;
            break
        end
    end
%         if ~collision_check(q/pi *180 , 0, holes(:,index)')
%             break
%         end
%     end
end
end

