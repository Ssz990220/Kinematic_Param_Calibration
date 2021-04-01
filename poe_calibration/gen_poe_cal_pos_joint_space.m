function [qs, p_measures, X_holes, Ts_end]= gen_poe_cal_pos_joint_space(n_points, robot, r)
%GEN_POE_CAL_POS_JOINT_SPACE Summary of this function goes here
%   Detailed explanation goes here
qs = (rand(n_points, 6)-0.5) * pi * 2;
p_measures = (rand(3,n_points) - 0.5) * r * 2;
Ts_end = robot.fkine(qs).double();
X_holes = zeros(3, n_points);
for i = 1:n_points
    X_holes4 = Ts_end(:,:,i) * [p_measures(:,i);1];
    X_holes(:,i) = X_holes4(1:3);
end
end

