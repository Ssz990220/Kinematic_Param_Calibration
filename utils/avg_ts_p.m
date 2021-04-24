function [Ts, ps] = avg_ts_p(Ts, p_measure, n_points)
%AVG_TS_P Summary of this function goes here
%   Detailed explanation goes here
ps = avg_p(p_measure, n_points);
Ts = avg_ts(Ts, n_points);
end

