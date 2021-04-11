function view_holes(Ts, scale, new_figure)
%VIEW_HOLES Summary of this function goes here
%   Visualize holes on the standard workpiece
    if nargin < 3
        figure
        hold on
        axis equal
    else
        if new_figure
            figure
            hold on
            axis equal
        end
    end
    for i = 1:size(Ts,3)
        t = Ts(:,:,i);
        origin = t(1:3,4);
        x = t(1:3,1)*scale;
        y = t(1:3,2)*scale;
        z = t(1:3,3)*scale;
        xq = quiver3(origin(1),origin(2),origin(3),x(1),x(2),x(3),'LineWidth',2);
        xq.Color = 'y';
        xq.AutoScale = 'off';
        xq.LineWidth = 2;
        yq = quiver3(origin(1),origin(2),origin(3),y(1),y(2),y(3),'LineWidth',2);
        yq.Color = 'm';
        yq.AutoScale = 'off';
        yq.LineWidth = 2;
        zq = quiver3(origin(1),origin(2),origin(3),z(1),z(2),z(3),'LineWidth',2);
        zq.Color = 'r';
        zq.AutoScale = 'off';
        zq.LineWidth = 2;
    end
end

