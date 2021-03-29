function T_holes = gen_random_hole_pos(n_holes, x_low, x_high, y_low, y_high, z_low, z_high)
%GEN_RANDOM_HOLE_POS Summary of this function goes here
%   Detailed explanation goes here
if nargin == 1
    x_low = 500;
    x_high = 1300;
    y_low = -500;
    y_high = 500;
    z_low = 800;
    z_high = 1300;
end
T_holes = repmat(eye(4),1,1,n_holes);
T_holes_x = randi([x_low,x_high],1,n_holes);
T_holes_y = randi([y_low,y_high],1,n_holes);
T_holes_z = randi([z_low,z_high],1,n_holes);
T_holes(1:3,4,:) = [T_holes_x;T_holes_y;T_holes_z];
end

