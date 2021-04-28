function [qs, T_balls, init_O] = gen_optimal_config(threshold)
%GEN_OPTIMAL_CONFIG Summary of this function goes here
%   Detailed explanation goes here
%% Random pose optimization
init_O = 0;
while init_O < threshold
% For ball %
n_balls = 1;

n_measure_each_ball = 64;
rand_measure_pose = true;
rand_pose = true;
% For measure %
r =10;
z_angle = 45;
shift_level = 50;
% Generate cube position
T_balls = gen_ball_pos(n_balls);
if rand_measure_pose
    [~, p_measures, qs, Fail] = gen_ball_measure_pos_collision_check(T_balls, r,z_angle,shift_level, n_measure_each_ball, rand_pose);
else
    for i = 1:n_balls
        row =8;
        column = 8;
        [Ts(:,:,((i-1)*n_measure_each_ball+1):i*n_measure_each_ball),p_measures(:,((i-1)*n_measure_each_ball+1):i*n_measure_each_ball)] = gen_eye_calibration_pos(T_balls(:,:,i), r, row, column, z_angle, shift_level);
    end
end
if Fail
    continue
end

init_O = get_O_multi_balls(qs, p_measures, 1, 64, 1);
fprintf('init_O is %.4f\n',init_O);
end

end

