function T_cubes_origin = gen_cube_location(n_cubes, angle_apart, z_cubes_origin, x_cubes_origin)
%GEN_CUBE_LOCATION Summary of this function goes here
%   Detailed explanation goes here
T_cubes_origin = zeros(4,4,n_cubes);
base_omega = 0 - angle_apart * (n_cubes - 1) / 2;
index = 0 : n_cubes-1;
omega = index * angle_apart + base_omega;
R = @(z) [cos(z), -sin(z), 0, 0; sin(z), cos(z), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
for i = 1:n_cubes
    T_cubes_origin(:,:,i) = R(omega(i)/180*pi)*transl([x_cubes_origin(i),0,z_cubes_origin(i)])*R(-45/180*pi);
end

end

