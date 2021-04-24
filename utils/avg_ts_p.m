function [avg_Ts, avg_p] = avg_ts_p(Ts, p_measure, n_points)
%AVG_TS_P Summary of this function goes here
%   Detailed explanation goes here
n = size(p_measure,2);
avg_p = zeros(3, n_points);
avg_Ts = zeros(4,4,n_points);
n_measures = n/n_points;
for i = 1:n_points
    p_measure_local = zeros(3,n_measures);
    Ts_local = zeros(4,4,n_measures);
    for j = 1:n_measures
        p_measure_local(:,j) = p_measure(:,i+(j-1)*n_points);
        Ts_local(:,:,j) = Ts(:,:,i+(j-1)*n_points);
    end
    avg_p(:,i) = mean(p_measure_local,2);
    avg_Ts(:,:,i) = avg_ts(Ts_local);
end    
end

