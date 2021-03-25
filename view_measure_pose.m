function view_measure_pose(T,p_measure)
    scale = 0.01;
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

