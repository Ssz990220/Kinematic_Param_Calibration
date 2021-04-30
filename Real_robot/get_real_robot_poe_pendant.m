function robot_poe = get_real_robot_poe_pendant(Ts, qs, robot_poe)
%GET_REAL_ROBOT_POE_PENDANT Summary of this function goes here
%   Detailed explanation goes here
counter = 0;
type = 2;
n_points = size(Ts,3);
iter = 0;
threshold = 1e-11;
while 1
    tic;
    Ts_nominal = zeros(size(Ts));
    for i = 1:n_points
        Ts_nominal(:,:,i) = robot_poe.fkine(qs(i,:));
    end
    [error, delta_poe] = kinematic_calibration_poe_absolute(robot_poe, qs,Ts, Ts_nominal, n_points, type);
    old_links = robot_poe.links;
    old_gst = robot_poe.g_st_poe;
    robot_poe.update_poe(delta_poe, type);
    time = toc;
    link_update = max(max(abs(old_links - robot_poe.links)));
    gst_update = max(abs(old_gst - robot_poe.g_st_poe));
    fprintf('Iteration %d takes time %.4f, error is %.10f, update is %.10f \n',[iter, time, error, link_update]);
    iter = iter + 1;
    counter = counter + 1;
    if error < threshold || ( iter > 50) || (link_update < 1e-9) %|| counter == 2
        break
    end
end
end

