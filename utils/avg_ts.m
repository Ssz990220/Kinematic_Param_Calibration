function avg= avg_ts(Ts)
%AVG_TS Summary of this function goes here
%   Detailed explanation goes here
n = size(Ts,3);
twists = zeros(6,n);
for i = 1:n
    se3 = MatrixLog6(Ts(:,:,i));
    twists(:,i) = se3ToVec(se3);
end
avg_twist = mean(twists,2);
avg = MatrixExp6(VecTose3(avg_twist));
end

