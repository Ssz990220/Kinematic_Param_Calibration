function p_measures= inverse_p_measures(T_balls,qs)
%INVERSE_P_MEASURES Summary of this function goes here
%   Detailed explanation goes here
robot = my_new_dh_robot();
n = size(qs,1);
Ts_end = robot.fkine(qs).double();
p_measures = zeros(3,n);
for m = 1:n
    p_measure_global = T_balls(1:3,4) - Ts_end(1:3,4,m);
    p_measure_tool = Ts_end(1:3,1:3,m)'*p_measure_global;
    p_measures(:,m) = p_measure_tool;
end

end

