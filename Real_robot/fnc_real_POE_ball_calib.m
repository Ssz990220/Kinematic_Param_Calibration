function fnc_real_POE_ball_calib(p_measures, qs, n_balls, n_measure_each_ball)
%FNC_REAL_POE_BALL_CALIB Summary of this function goes here
%   Detailed explanation goes here
type = 1;
global robot_poe
iter = 0;
threshold = 1e-11;
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
    if error < threshold || ( iter > 100) || (link_update < 1e-11) 
        break
    end
end
end

