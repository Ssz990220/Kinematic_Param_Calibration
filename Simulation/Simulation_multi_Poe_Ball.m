clear;
clc;
%% Prepare robot
robot = my_new_dh_robot();
R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];
robot_poe = my_poe_robot(T_tool, true, 0.005,0.05, true,0,0.1,false);
%(T_tool, add_joint_shift, omega_shift_level, q_shift_level, add_base_shift, base_shift_omega, base_shift_q, add_angle_noise, angle_error_level, angle_error_decay)
%% Initial Error
error = 0;
counter = 0;
n_test = 10;
T_act = zeros(4,4,n_test);
T_mea = zeros(4,4,n_test);

for i = 1:n_test
    pose = rand(1,6);
    T_act(:,:,i) = robot.fkine(pose).double();
    T_mea(:,:,i) = robot_poe.fkine(pose);
end

for i = 1:n_test
    for j = (i+1):n_test
        counter = counter+1;
        error = error + norm(norm(T_act(1:3,4,i)-T_act(1:3,4,j))-norm(T_mea(1:3,4,i)-T_mea(1:3,4,j)));
    end
end
error_init = error / counter;

%% parameters
% For ball %
n_balls = 1;
n_measure_each_ball = 64;
rand_pose = true;
% For measure %
r = 50;
z_angle = 45;
threshold = 1e-11;
type = 1;
% noise %
noise_level = 0.03;
add_noise = true;
% For visualization %
visualize_hole = false;
visualize_pose = false;
n_iter = 1;

for Iter = 1:n_iter
%% Generate cube position
T_balls = gen_ball_pos(n_balls);
%% Generate measuring pose and measure data
%     z_angle = z_angle_list(mod(iter,size(z_angle_list,2))+1);
[Ts, p_measures] = gen_ball_measure_pos(T_balls, r,z_angle,10, n_measure_each_ball);

if visualize_hole
    view_holes(T_balls,100,true);
    view_measure_pose(Ts, p_measures, 100, false);
end
% robot.plot(zeros(1,6));
%% ikine
tic;
qs = zeros(n_balls*n_measure_each_ball, 6);
Ts(1:3,4,:) = Ts(1:3,4,:)/1000;
if rand_pose
    for i = 1:n_balls*n_measure_each_ball
        qss = exp_ikine(Ts(:,:,i),zeros(6,1),1)';
        qss = qss(any(qss,1),:);
        qs(i,:) = qss(randi([1,size(qss,2)]),:);
        if qs(i,:) == zeros(6,1)
            error('ikine result not found');
        end
    end
else
    for i = 1:n_balls*n_measure_each_ball
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
    robot_view_generate_pose(robot,qs,2);
end

%% Add noise
if add_noise
    noise = normrnd(0, noise_level, size(p_measures));
    p_measures = p_measures + noise;
end
%% Calibration
iter = 1;
while 1
    tic;
    [error, delta_poe] = multi_ball_kinematic_calibration_poe(robot_poe, qs, p_measures, type, n_balls, n_measure_each_ball);
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    fprintf('Iteration %d \t takes time %.4f,\t error is %.12f \t links update is %.10f \t g_st update is %.10f \n',[iter, time, error, link_update, gst_update]);
    iter = iter + 1;
    if error < threshold || ( iter > 50) || (link_update < 1e-11) 
        break
    end
end
%% Validation
error = 0;
counter = 0;
T_act = zeros(4,4,n_test);

for i = 1:n_test
    pose = rand(1,6);
    T_act(:,:,i) = robot.fkine(pose).double();
    T_mea(:,:,i) = robot_poe.fkine(pose);
end

for i = 1:n_test
    for j = (i+1):n_test
        counter = counter+1;
        error = error + norm(norm(T_act(1:3,4,i)-T_act(1:3,4,j))-norm(T_mea(1:3,4,i)-T_mea(1:3,4,j)));
    end
end
error = error / counter;
   

fprintf("Initial error is %.2f, after calibration, error is %.10f \n",[error_init, error]);

%% End
end