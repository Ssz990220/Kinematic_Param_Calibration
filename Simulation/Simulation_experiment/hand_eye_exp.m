clear;
clc;
n_times = 25;
noise_levels = [0.2,1;0.1,0.5;0.05,0.1];
r_c =[2,4;4,4;4,8];
rs = [0,10,15,20];
z_angles = [15,30];
shift_levels = [0,1,5,10];
result = [];
total_iter = 960;
iter = 0;
tic;
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
                    error = 0;
                    for i = 1:n_times
                        error = error + fnc_sim_eye_calib(r, row, column, z_angle, shift_level, measure_noise, Ts_noise);
                    end
                    error = error /n_times;
                    result = [result; row, column, r, shift_level, z_angle, measure_noise, Ts_noise, error];
                    clc;
                    iter = iter + a;
                    time = toc;
                    eta = time/iter * (total_iter - iter);
                    min = eta/60;
                    fprintf('%.2f % done, ETA: %.2f min \n',[iter/total_iter*100, min]);
                end
            end
        end
    end
end

filename = '.\Simulation\Simulation_experiment\result_raw.xlsx';
writematrix(result, 'Sheet',1);
                    
                    
% for i = 1:n_times
% %                                                            r     row   column  z_angle  shift_level   measure_noise    Ts_noise
%     error = error + fnc_sim_eye_calib(20,    2,       4,            15,          10,                 0.2,                        1);
% end
% error/n_times