clear;
clc;
link1 = Link('d',495,'a',175,'alpha',pi/2);
link2 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
link3 = Link('d',0,'a',175,'alpha',pi/2);
link4 = Link('d',960,'a',0,'alpha',-pi/2);
link5 = Link('d',0,'a',0,'alpha',pi/2);
link6 = Link('d',135,'a',0,'alpha',0);
robot_without_tool = SerialLink([link1, link2, link3, link4, link5, link6]);
% robot_with_tool.plot(zeros(1,6));
qs = load('qs5.txt');
qs = qs(1:4,:);

%% load measure position
p_measure = [-2.25387, 41.1774, -2.36082;
            6.31666, 41.9971, -3.89512;
            -12.1279, 23.9648, 11.5311;
            11.3552, 26.348, 10.7816]';   
        
%%
Ts_without_tool= robot_without_tool.fkine(qs).double();
Tool_T = hand_eye_calibration(Ts_without_tool, p_measure)


% Tool = [eye(3),[0,0,270]';
%         zeros(1,3),1];
robot_with_tool = SerialLink([link1, link2, link3, link4, link5, link6],'tool',Tool_T);
pos_eff = robot_with_tool.fkine(qs(1,:)).double();
ball_pos = pos_eff * [eye(3),qs(1,:)';zeros(1,3),1];
ball_pos = ball_pos(1:3,4)
