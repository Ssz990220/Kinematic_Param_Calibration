function qs = gen_hole_measure_pos_cone(holes, n_measures, angle_apart)
%GEN_HOLE_MEASURE_POS_RAND Summary of this function goes here
%   Detailed explanation goes here
r = 0;
n_holes = size(holes,2);
qs = zeros(n_measures * n_holes + 1, 6);
figure
hold on
axis equal
z_angles = -angle_apart:2*angle_apart/(n_measures-1):angle_apart;
for i = 1:n_holes*n_measures
    T_hole = eye(4);
    index_h = mod(i, n_holes);
    if index_h == 0
        index_h = n_holes;
    end
    index_m =floor((i-1)/n_holes)+1;
%     if index_m == 0
%         index_m = n_measures;
%     end
    T_hole(1:3,4) = holes(:,index_h);
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
%     while 1
        angle_y = 75;
        angle_z = z_angles(index_m);
        rotation_y = T_y(angle_y/180*pi);
        rotation_z = T_z(angle_z/180*pi);
        T_measure=T_hole*rotation_z*rotation_y*T_x*T_add;
        T_meter = T_measure;
        T_meter(1:3,4) = T_meter(1:3,4)/1000;
%%
%         qs_= exp_ikine(T_meter, qs(i,:)',1);
%         qs_ = qs_(:,qs_(4,:)>(-120/180*pi));
%         qs_ = qs_(:,qs_(6,:)>(-100/180*pi));
%         q_dis = vecnorm(qs_ - qs(i,:)',2);
%         [~,min_index] = min(q_dis);
%         q = qs_(:,min_index);
%%
%         q = exp_ikine(T_meter, qs(i,:)',2);
%         if q(4)<-120/180*pi
%             q(4) = q(4) + 360;
%         end
%         if q(6) < -100/180*pi
%             q(6) = q(6) + 360;
%         end
%%
        qs_ = exp_ikine(T_meter, qs(i,:)',1);
        mask = [];
%         close all;
        for j = 1:size(qs_,2)
            if collision_check(qs_(:,j)/pi *180 , 0, holes(:,index_h)',0)
                mask = [mask, j];
            end
        end
        qs_(:,mask) = [];
        qs_ =  qs_(:,qs_(1,:)>(-120/180*pi));
        qs_ =  qs_(:,qs_(1,:)<(120/180*pi));
%         qs_ = qs_(:,qs_(4,:)>(-120/180*pi));
%         qs_ = qs_(:,qs_(6,:)>(-100/180*pi));
%         q_dis = vecnorm(qs_ - qs(i,:)',2);
%         [~,min_index] = min(q_dis);
        q = qs_(:,randsample(size(qs_,2),1));
%%
        p_measure= -[0,0,r]';
%         if q(4)>-120 || q(6)>-100
        view_measure_pose(T_measure, p_measure, 10, false);
        qs(i+1,:) = q;
%             break
%         end
%     end
%         if ~collision_check(q/pi *180 , 0, holes(:,index)')
%             break
%         end
%     end
end
end

