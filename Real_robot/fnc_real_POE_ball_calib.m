function robot_poe = fnc_real_POE_ball_calib(p_measures, qs, n_balls, n_measure_each_ball, robot_poe)
%FNC_REAL_POE_BALL_CALIB Summary of this function goes here
%   Detailed explanation goes here
type = 1;
[error, robot_poe] = multi_ball_kinematic_calibration_poe_optim(robot_poe, qs(mask,:), p_measures(:,mask), type, n_balls, length(mask));
end

