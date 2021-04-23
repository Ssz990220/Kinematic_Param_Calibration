function T_avg = SE3_avg(T_list)
    %AVG_TS Summary of this function goes here
    %   Detailed explanation goes here
    n = size(T_list,3);
    twists = zeros(6,n);
    for i = 1:n
        se3 = MatrixLog6(T_list(:,:,i));
        twists(:,i) = se3ToVec(se3);
    end
    avg_twist = mean(twists,2);
    T_avg = MatrixExp6(VecTose3(avg_twist));
end