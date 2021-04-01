clear;
clc;
n_points =32;
r = 200;
robot = my_new_dh_robot();
[qs, p_measures, X_holes, Ts] = gen_poe_cal_pos_joint_space(n_points, robot, r);
T_holes = repmat(eye(4),1,1,n_points);
for i = 1:n_points
    T_holes(1:3,4,i) = X_holes(:,i);
end
%% view holes and p_measure
figure
hold on
axis equal
view_holes(T_holes,50,false);
view_measure_pose(Ts, p_measures, 50, false);