clear;
clc;
n_holes = 20;
T_holes = repmat(eye(4),1,1,n_holes);
T_holes_x = randi([500,1000],1,n_holes);
T_holes_y = randi([-500,500],1,n_holes);
T_holes_z = randi([800,1300],1,n_holes);
T_holes(1:3,4,:) = [T_holes_x;T_holes_y;T_holes_z];
measure_per_point = 1;
r = 20;
z_angle = 45;
[Ts, p_measures] = gen_poe_cal_pos(T_holes, measure_per_point, r, z_angle, 10);
%% view holes and p_measure
figure
hold on
axis equal
view_holes(T_holes,10,false);
view_measure_pose(Ts, p_measures, 10, false);

%% Visualize on real robot
robot = my_new_dh_robot();
qs = robot.ikine(Ts);
robot_view_generate_pose(robot, qs);