clear;
clc;
link1 = Link('d',495,'a',175,'alpha',pi/2);
link2 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
link3 = Link('d',0,'a',175,'alpha',pi/2);
link4 = Link('d',960,'a',0,'alpha',-pi/2);
link5 = Link('d',0,'a',0,'alpha',pi/2);
link6 = Link('d',135,'a',0,'alpha',0);
R =eye(3);
Tool = [R,[0,0,370]';
        zeros(1,3),1];
robot_with_tool = SerialLink([link1, link2, link3, link4, link5, link6],'tool',Tool);
robot_without_tool = SerialLink([link1, link2, link3, link4, link5, link6]);
% qs = [-1.64 9.84 -18.19 2.88 -21.18 -1.99]/180*pi;
% qs(2) = -qs(2);
% qs(3) = -qs(3);
% qs(5) = -qs(5);
% robot_with_tool.fkine(qs);
% Tool = [R,[0,0,276]';
%         zeros(1,3),1];
    


T_obj = [9.30724 -19.0494 -4.02572]';       % Change this line every time
T_obj = [T_obj;1];

SE3_lj = convert_real_robot_pos([1278.25 -5.73 1415.92 0.47329 -0.13596 0.86878 -0.05224]);
R_ = [-1,0,0;0,1,0;0,0,-1]';
Tool_ = [R_,[0,0,370]';
        zeros(1,3),1];
Ball_pos = SE3_lj *Tool_ * T_obj;
Ball_pos = Ball_pos(1:3);
T_ball = [eye(3), Ball_pos;zeros(1,3),1];
row = 4;
column = 8;
[Ts,p_measure] = gen_eye_calibration_sim(T_ball, 0, row, column, 15,10);
% figure
% hold on
% axis equal
% for i =1:row*column
%     view_measure_pose(Ts(:,:,i),p_measure(:,i));
% end
qs_measure = robot_with_tool.ikine(Ts);
for i = 1:row*column
    qs_measure(:,2) = -qs_measure(:,2);
    qs_measure(:,3) = -qs_measure(:,3);
    qs_measure(:,5) = -qs_measure(:,5);
end
qs_measure_reorder = ZigZag(qs_measure,row,column);
% for i = 1:row*column
%     robot_with_tool.plot(qs_measure_reorder(i,:));
%     pause(0.5);
% end
qs_measure_reorder = qs_measure_reorder * 180 / pi;
save 'qs_measure_7.txt' qs_measure_reorder -ascii