function Ts = convert_real_robot_pos(last_link_pos)
%CONVERT_REAL_ROBOT_POS Summary of this function goes here
%   This function convert last joint posture read on the pendant to a
%   standard SE3 matrix
Ts = zeros([4,4,size(last_link_pos,1)]);
for i = 1:size(last_link_pos,1)
    T = last_link_pos(i,1:3);
    q = last_link_pos(i,4:7);
    R =[2*q(1)^2-1+2*q(2)^2, 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(2)*q(4)+q(1)*q(3));
            2*(q(2)*q(3)+q(1)*q(4)), 2*q(1)^2-1+2*q(3)^2, 2*(q(3)*q(4)-q(1)*q(2));
            2*(q(2)*q(4)-q(1)*q(3)), 2*(q(3)*q(4)+q(1)*q(2)), 2*q(1)^2-1+2*q(4)^2];
    Ts(:,:,i) = [R,T';zeros(1,3),1];
end
end

