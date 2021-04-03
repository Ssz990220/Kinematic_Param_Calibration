dis_holes = 100;
n_holes_each_line = 3;
edge_length = 400;
n_cubes = 3;
cube_type = [1,2,1];
z_cubes = [800, 2000, 800];
x_cubes = [1000,1000,1000];
angle_apart = 90;
T_cubes = gen_cube_location(3, angle_apart, z_cubes, x_cubes);
cubes = [];
for i = 1:n_cubes
    cube = standard_cubic_workpiece(T_cubes(:,:,i), cube_type(i), n_holes_each_line, dis_holes, edge_length);
    cubes = [cubes, cube];
end
cubes_array = Cubes_array(cubes);
T_holes = cubes_array.get_all_Ts();
[Ts, p_measures] = gen_cubic_measure_pos(cubes_array, 20,45,10);
%% view holes and p_measure
figure
hold on
axis equal
xlabel 'x'
ylabel 'y'
zlabel 'z'
view_holes(T_holes,50,false);
view_measure_pose(Ts, p_measures, 50, false);