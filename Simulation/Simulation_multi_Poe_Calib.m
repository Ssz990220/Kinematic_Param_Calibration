clear;
clc;
%% Prepare robot
robot = my_new_dh_robot();

R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe = my_poe_robot(T_tool, true, 0.05,1, false,0.001,0.2,false);

%% parameters
n_holes_row = 3;
n_holes_cube = n_holes_row^2*3;
n_cubes = 3;
n_holes = n_cubes * n_holes_cube;
T_holes = zeros(4,4,n_holes);
T_holes(:,:,1:n_holes_cube)= gen_random_hole_pos(n_holes_cube,900,1300,-200,200,700,1100);
T_holes(:,:,n_holes_cube + 1:n_holes_cube*2) = gen_random_hole_pos(n_holes_cube, -200,200,900,1300,700,1100);
T_holes(:,:,n_holes_cube*2+1:n_holes_cube*3) = gen_random_hole_pos(n_holes_cube,-200,200,-1300,-900,700,1100);
T_holes(1:3,1:3,n_holes_cube+1:n_holes_cube*2) = repmat([0,1,0;-1,0,0;0,0,1]',1,1,n_holes_cube);
T_holes(1:3,1:3,n_holes_cube*2+1:n_holes_cube*3) = repmat([0,-1,0;1,0,0;0,0,1]',1,1,n_holes_cube);

measure_per_point = 1;
r = 200;
z_angle = 15;
threshold = 1e-11;
iter = 1;
type = 1;

%% Generate measuring pose and measure data
%     z_angle = z_angle_list(mod(iter,size(z_angle_list,2))+1);
[Ts, p_measures] = gen_poe_cal_pos(T_holes, measure_per_point, r, z_angle,10);
view_holes(T_holes,100,true);
view_measure_pose(Ts, p_measures, 100, false);
qs = robot.ikine(Ts);   % n_points by 6
x_true = zeros(3,n_holes);
for i=1:n_holes
    x_true(:,i) = T_holes(1:3,4,i);
end

%% Initial Error
error = 0;
n_test = 100;
for i = 1:n_test
    pose = rand(1,6);
    T1 = robot.fkine(pose).double();
    T2 = robot_poe.fkine(pose);
    error = error + norm(T1(1:3,4)-T2(1:3,4));
end
error_init = error / n_test;

%% Add noise
noise_level = 0.05;
noise = (rand(size(p_measures))-0.5)*2*noise_level;
p_measures = p_measures + noise;
%% Calibration

while 1
    tic;
    [error, delta_poe] = multi_kinematic_calibration_poe(robot_poe, qs, p_measures, x_true, type, n_holes_cube, n_cubes);
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    fprintf('Iteration %d \t takes time %.4f,\t error is %.12f \t links update is %.10f \t g_st update is %.10f \n',[iter, time, error, link_update, gst_update]);
    iter = iter + 1;
    if error < threshold
        break
    end
    if iter > 50
        break
    end
end
%% Validation
error = 0;
n_test = 100;
for i = 1:n_test
    pose = rand(1,6);
    T1 = robot.fkine(pose).double();
    T2 = robot_poe.fkine(pose);
    error = error + norm(T1(1:3,4)-T2(1:3,4));
end
error = error / n_test;

fprintf("Initial error is %.2f, after calibration, error is %.10f \n",[error_init, error]);