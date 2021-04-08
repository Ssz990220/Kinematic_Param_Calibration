function view_measure_pose(Ts,p_measures, scale, new_figure)
% Visualize the measuring pose and generated measure data.
% If all generate data point to the same location, then it is a valid result
if nargin < 4
    figure
    hold on
    axis equal
else
    if new_figure
        figure
        hold on
        axis equal
    else
        hold on
    end
end
for i = 1:size(Ts,3)
    t = Ts(:,:,i);
    p_measure = p_measures(:,i);
    origin = t(1:3,4);
    x = t(1:3,1)*scale;
    y = t(1:3,2)*scale;
    z = t(1:3,3)*scale;
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
end

