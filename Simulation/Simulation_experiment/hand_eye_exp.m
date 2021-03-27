% clear;
% clc;
n_times = 20;
noise_levels = [0.2,1;0.1,0.5;0.05,0.1];
r_c =[2,4;4,4;4,8];
rs = [0,10,15,20];
z_angles = [15,30];
shift_levels = [0,1,5,10];
result = [];
total_iter = 960;
iter = 0;
time_consumed =0;
for a = 1:size(r_c,1)
    row = r_c(a,1);
    column = r_c(a,2);
    for b = 1:size(rs,2)
        r = rs(b);
        for c = 1:size(shift_levels,2)
            shift_level = shift_levels(c);
            for d = 1:size(z_angles,2)
                z_angle = z_angles(d);
                for e = 1:size(noise_levels,1)
                    measure_noise = noise_levels(e,1);
                    Ts_noise = noise_levels(e,2);
                    error = zeros([1,n_times]);
                    tic;
                    parfor i = 1:n_times
                        error(i) = fnc_sim_eye_calib(r, row, column, z_angle, shift_level, measure_noise, Ts_noise);
                    end
                    error = mean(error);
                    result = [result; row, column, r, shift_level, z_angle, measure_noise, Ts_noise, error];
                    clc;
                    iter = iter + 3^(a-1);
                    time = toc;
                    time_consumed = time_consumed + time;
                    eta = time_consumed/iter * (total_iter - iter);
                    min = eta/60;
                    fprintf('%.3f percent done, ETA: %.2f min \n',[iter/total_iter*100, min]);
                end
            end
        end
    end
end


%% Save result

filename = '.\Simulation\Simulation_experiment\result_raw.xlsx';
writematrix( result, filename, 'Sheet',1);
                    
                    
%% Visualize result
figure
axis equal
