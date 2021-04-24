function avg= avg_ts(Ts, n_points)
%AVG_TS Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 2
        avg = avg_one_ts(Ts);
    else
        n = size(Ts,3);
        avg = zeros(4,4,n_points);
        n_measures = n/n_points;
        for i = 1:n_points
            Ts_local = zeros(4,4,n_measures);
            for j = 1:n_measures
                Ts_local(:,:,j) = Ts(:,:,i+(j-1)*n_points);
            end
            avg(:,:,i) = avg_one_ts(Ts_local);
        end
    end
end

function avg_T = avg_one_ts(Ts)
    n_Ts = size(Ts,3);
    twists = zeros(6,n_Ts);
    for i = 1:n_Ts
        se3 = MatrixLog6(Ts(:,:,i));
        twists(:,i) = se3ToVec(se3);
    end
    avg_twist = mean(twists,2);
    avg_T = MatrixExp6(VecTose3(avg_twist));
end


