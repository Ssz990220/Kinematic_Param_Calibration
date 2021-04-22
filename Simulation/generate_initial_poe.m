clear;
clc;
% Prepare robot
robot = my_new_dh_robot();
R_ = [-1,0,0;0,1,0;0,0,-1]';
T_tool= [R_,[0,0,370]';
        zeros(1,3),1];
    
error_init = 5;
while error_init>0.2
robot_poe = my_poe_robot(T_tool, true, 0.005,0.05, true,0,0.1,false);
%(T_tool, add_joint_shift, omega_shift_level, q_shift_level, add_base_shift, base_shift_omega, base_shift_q, add_angle_noise, angle_error_level, angle_error_decay)
% Initial Error
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
end

save fake_robot robot_poe