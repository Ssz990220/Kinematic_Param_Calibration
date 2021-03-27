clear;
clc;
T0 = [eye(3),[1000,0,1300]';
            zeros(1,3),1];
dis =  50 * ones(6,1);
direction = [1,0,0]';
measure_per_point = 3;
n_points = 7;
r = 50;
[Ts, T_holes, p_measures] = gen_poe_cal_pos(T0, direction, dis, measure_per_point, n_points, r, 15, 10);
figure
hold on
axis equal
view_holes(T_holes,10,false);
view_measure_pose(Ts, p_measures, 10, false);
robot = my_new_dh_robot();
qs = robot.ikine(Ts);
robot_view_generate_pose(robot, qs);