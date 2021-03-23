addpath('../');
addpath('../mr');
clear;
clc;
T0 = [eye(3),[500,0,200]';
            zeros(1,3),1];
dis =  50 * ones(6,1);
direction = [1,0,0]';
measure_per_point = 3;
n_points = 7;
r = 200;
[Ts, T_holes, p_measures] = gen_poe_cal_pos_sim(T0, direction, dis, measure_per_point, n_points, r);
figure
hold on
for i = 1:n_points
  draw_arrow_3d(T_holes(:,:,i),[0,0,0]');  
end
for i=1:size(Ts,3)
    draw_arrow_3d(Ts(:,:,i),p_measures(:,i));
end

function draw_arrow_3d(T,p_measure)
    scale = 10;
    origin = T(1:3,4);
    x = T(1:3,1)*scale;
    y = T(1:3,2)*scale;
    z = T(1:3,3)*scale;
    xq = quiver3(origin(1),origin(2),origin(3),x(1),x(2),x(3));
    xq.Color = 'y';
    xq.AutoScale = 'off';
    xq.LineWidth = 2;
    yq = quiver3(origin(1),origin(2),origin(3),y(1),y(2),y(3));
    yq.Color = 'm';
    yq.AutoScale = 'off';
    yq.LineWidth = 2;
    zq = quiver3(origin(1),origin(2),origin(3),z(1),z(2),z(3));
    zq.Color = 'r';
    zq.AutoScale = 'off';
    zq.LineWidth = 2;
    target_point = (x * p_measure(1) + y * p_measure(2) + z * p_measure(3))/scale;
    target = quiver3(origin(1),origin(2),origin(3),target_point(1), target_point(2), target_point(3));
    target.Color = 'c';
    target.AutoScale = 'off';
    target.LineWidth = 2;
end