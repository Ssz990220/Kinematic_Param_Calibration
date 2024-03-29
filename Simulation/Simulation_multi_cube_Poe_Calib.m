clear;
clc;
%% Prepare robot
robot = my_new_dh_robot();

R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe = my_poe_robot(T_tool, true, 0.005,0.01, false,0.001,0.2,false);

%% parameters
% For cube position %
dis_holes = 50;
n_holes_each_line = 4;
n_cubes = 1;
measure_times = 1;
rand_pose = true;
% For measure %
r = 50;
z_angle = 45;
threshold = 1e-11;
type = 1;
% noise %
noise_level = 0.01;
add_noise = false;
% For visualization %
visualize_hole = false;
visualize_pose = false;
n_iter = 1;
for iter = 1:n_iter
%% Generate cube position
edge_length = dis_holes * (n_holes_each_line + 1);
angle_apart = 180/n_cubes;
n_holes_cube = n_holes_each_line^2 * 3;
n_holes = n_holes_cube * n_cubes * measure_times;
cube_type = ones(1,n_cubes * measure_times);
z_cubes = randi([450,550],1,n_cubes);
x_cubes = randi([850,950],1,n_cubes);
T_cubes = gen_cube_location(n_cubes, angle_apart, z_cubes, x_cubes);
T_cubes = repmat(T_cubes, 1, 1, measure_times);
cubes = [];
for i = 1:n_cubes * measure_times
    cube = standard_cubic_workpiece(T_cubes(:,:,i), cube_type(i), n_holes_each_line, dis_holes, edge_length);
    cubes = [cubes, cube];
end
cubes_array = Cubes_array(cubes);
T_holes = cubes_array.get_all_Ts();
%% Generate measuring pose and measure data
%     z_angle = z_angle_list(mod(iter,size(z_angle_list,2))+1);
[Ts, p_measures] = gen_cubic_measure_pos(cubes_array, r,z_angle,10);
if visualize_hole
    view_holes(T_holes,10,true);
    view_measure_pose(Ts, p_measures, 10, false);
end
% robot.plot(zeros(1,6));
%% ikine
tic;
qs = zeros(n_holes, 6);
Ts(1:3,4,:) = Ts(1:3,4,:)/1000;
if rand_pose
    for i = 1:n_holes
        qss = exp_ikine(Ts(:,:,i),zeros(6,1),1)';
        qss = qss(any(qss,1),:);
        qs(i,:) = qss(randi([1,size(qss,2)]),:);
        if qs(i,:) == zeros(6,1)
            error('ikine result not found');
        end
    end
else
    for i = 1:n_holes
        if i == 1
            qs(i,:) = exp_ikine(Ts(:,:,i),zeros(6,1),2)';
        else
            qs(i,:) = exp_ikine(Ts(:,:,i),qs(i-1,:)',2)';
        end
    end
end
% qs = robot.ikine(Ts);
time = toc;
fprintf('Inverse kinematics takes %.4f sec to complete\n',time);
if visualize_pose
    robot_view_generate_pose(robot,qs,0.1);
end
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


%% Initaial POE
x = robot_poe.output(1);

%% Add noise
if add_noise
    noise = normrnd(0, noise_level, size(p_measures));
    p_measures_ = p_measures + noise;
else
    p_measures_ = p_measures;
end
% Calibration
robot_poe = robot_poe.initialize(x,1);
errors = [error_init];
iter = 1;
while 1
    tic;
    [error, delta_poe] = multi_kinematic_calibration_poe(robot_poe, qs, p_measures_, x_true, type, n_holes_cube, n_cubes, measure_times);
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    %Test
    error = 0;
    n_test = 100;
    for i = 1:n_test
        pose = rand(1,6);
        T1 = robot.fkine(pose).double();
        T2 = robot_poe.fkine(pose);
        error_local = MatrixLog6(T1*TransInv(T2));
        error = error + norm(error_local);
    end
    error = error / n_test;
    %record
    errors = [errors,error];
    fprintf('Iteration %d \t takes time %.4f,\t error is %.12f \t links update is %.10f \t g_st update is %.10f \n',[iter, time, error, link_update, gst_update]);
    iter = iter + 1;
    if error < threshold || ( iter > 50) || (link_update < 1e-9) 
        break
    end
end
% Validation
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

%% End
end