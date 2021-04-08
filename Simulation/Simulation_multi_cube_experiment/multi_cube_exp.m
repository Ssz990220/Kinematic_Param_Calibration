%% Parameter sets
% for cube position
dis_holes_s = [50,100,200];
n_holes_each_line_s = [2, 3, 4];
n_cubes_s = [3, 6, 12, 24, 36];
measure_times_s = [1, 3, 6, 12];
rand_pose_s = [true, false];
% For measure %
r = 50;
z_angle = 45;
threshold = 1e-11;
type = 1;
% noise %
noise_level_s = [0.03, 0.01];
add_noise = false;
% For visualization %
visualize_hole = false;
visualize_pose = false;
% For experiment
n_times = 32;

%% experiment
total_iter = (2^4 + 3^4 + 4^4) * sum(n_cubes_s) * sum(measure_times_s) * 3 * 2 * 2;
iter = 0;
options.r = r;
options.z_angle = z_angle;
options.add_noise = true;
options.threshold = 1e-11;
result = [];
time_consumed =0;
for d = 1:size(dis_holes_s,2)
    dis_holes = dis_holes_s(d);
    for n_h = 1:size(n_holes_each_line_s,2)
        n_holes_each_line = n_holes_each_line_s(n_h);
        for n_c = 1:size(n_cubes_s,2)
            n_cubes = n_cubes_s(n_c);
            for m = 1:size(measure_times_s,2)
                measure_times = measure_times_s(m);
                for ran = 1:size(rand_pose_s,2)
                    rand_pose = rand_pose_s(ran);
                    for no = 1:size(noise_level_s,2)
                        noise_level = noise_level_s(no);
                        options.noise_level = noise_level;
                        options.dis_holes = dis_holes;
                        options.n_holes_each_line = n_holes_each_line;
                        options.n_cubes = n_cubes;
                        options.measure_times = measure_times;
                        options.rand_pose = rand_pose;
                        errors = zeros([1,n_times]);
                        tic;
                        for i = 1:n_times
                            [~, error] = fnc_sim_multi_cube_poe_calib(options);
                            errors(i) = error;
                        end
                        error = mean(errors);
                        result = [result; dis_holes, n_holes_each_line, n_cubes, measure_times, rand_pose, noise_level, error];
                        clc;
                        iter = iter + n_holes_each_line ^ 4 * measure_times * n_cubes;
                        time = toc;
                        time_consumed = time_consumed + time;
                        eta = time_consumed/iter * (total_iter - iter);
                        min = eta/60;
                        fprintf('%.3f percent done, ETA: %.2f min \n',[iter/total_iter*100, min])
                    end
                end
            end
        end
    end
end
%% Save reuslt
filename = '.\Simulation\Simulation_multi_cube_experiment\result_raw.xlsx';
writematrix( result, filename, 'Sheet',1);