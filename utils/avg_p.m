function avg_p = avg_p(p_measure, n_points)
%AVG_P Summary of this function goes here
%   Detailed explanation goes here
if nargin < 2
    avg_p = mean(p_measure,2);
else
    n = size(p_measure,2);
    avg_p = zeros(3, n_points);
    n_measures = n/n_points;
    for i = 1:n_points
        p_measure_local = zeros(3,n_measures);
        for j = 1:n_measures
            p_measure_local(:,j) = p_measure(:,i+(j-1)*n_points);
        end
        avg_p(:,i) = mean(p_measure_local,2);
    end       
end
end

