addpath('../');
addpath('../mr');
Object_T = [eye(3),[500,0,200]';
            zeros(1,3),1];
r = 200;
Ts = gen_eye_calibration_pos(Object_T, r, 2, 4);
figure
hold on
draw_arrow_3d(Object_T,50);
for i=1:size(Ts,3)
    draw_arrow_3d(Ts(:,:,i),r);
end

function draw_arrow_3d(T,r)
    origin = T(1:3,4);
    x = T(1:3,1)*50;
    y = T(1:3,2)*50;
    z = T(1:3,3)*r;
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
end