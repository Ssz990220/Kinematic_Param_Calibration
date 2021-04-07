function [error_init,error] = fnc_sim_multi_cube_poe_calib(options)
%FNC_SIM_MULTI_CUBE_POE_CALIB Summary of this function goes here
%   Detailed explanation goes here
%% Build Robot
robot = my_new_dh_robot();

R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe = my_poe_robot(T_tool, true, 0.005,0.001, false,0.001,0.2,false);

%% Generate cube position
angle_apart = 360/options.n_cubes;
n_holes_cube = options.n_holes_each_line^2 * 3;
n_holes = n_holes_cube * options.n_cubes * options.measure_times;
cube_type = ones(1,options.n_cubes * options.measure_times);
z_cubes = repmat(randi([450,550],1,options.n_cubes),1,options.measure_times);
x_cubes = repmat(randi([850,950],1,options.n_cubes),1,options.measure_times);
T_cubes = gen_cube_location(options.n_cubes * options.measure_times, options.angle_apart, z_cubes, x_cubes);
cubes = [];
for i = 1:n_cubes * options.measure_times
    cube = standard_cubic_workpiece(T_cubes(:,:,i), cube_type(i), options.n_holes_each_line, options.dis_holes, options.edge_length);
    cubes = [cubes, cube];
end
cubes_array = Cubes_array(cubes);
T_holes = cubes_array.get_all_Ts();
%% Generate measuring pose and measure data
[Ts, p_measures] = gen_cubic_measure_pos(cubes_array, options.r,options.z_angle,10);
%% ikine
tic;
qs = zeros(n_holes, 6);
Ts(1:3,4,:) = Ts(1:3,4,:)/1000;
if options.rand_pose
    parfor i = 1:n_holes
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
if options.add_noise
    noise = normrnd(0, options.noise_level, size(p_measures));
    p_measures = p_measures + noise;
end
%% Calibration
iter = 1;
while 1
    tic;
    [error, delta_poe] = multi_kinematic_calibration_poe(robot_poe, qs, p_measures, x_true, type, n_holes_cube, n_cubes * measure_times);
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    fprintf('Iteration %d \t takes time %.4f,\t error is %.12f \t links update is %.10f \t g_st update is %.10f \n',[iter, time, error, link_update, gst_update]);
    iter = iter + 1;
    if error < options.threshold || ( iter > 50) || (link_update < 1e-9) 
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

end

