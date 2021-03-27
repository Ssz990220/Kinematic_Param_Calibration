function robot = my_new_dh_robot(tool)
%MY_DH_ROBOT Summary of this function goes here
%   Detailed explanation goes here
if nargin < 1
    R_ = [-1,0,0;0,1,0;0,0,-1]';
    tool= [R_,[0,0,370]';
            zeros(1,3),1];
end
link1 = Link('d',495,'a',175,'alpha',-pi/2);
link2 = Link('d',0,'a',900,'alpha',0,'offset',-pi/2);
link3 = Link('d',0,'a',175,'alpha',-pi/2);
link4 = Link('d',960,'a',0,'alpha',pi/2);
link5 = Link('d',0,'a',0,'alpha',-pi/2);
link6 = Link('d',135,'a',0,'alpha',0,'offset',pi);
robot = SerialLink([link1, link2, link3, link4, link5, link6],'tool',tool);
end

