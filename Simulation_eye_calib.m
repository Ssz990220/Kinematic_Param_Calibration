clear;
clc;
link1 = Link('d',495,'a',175,'alpha',pi/2);
link2 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
link3 = Link('d',0,'a',175,'alpha',pi/2);
link4 = Link('d',960,'a',0,'alpha',-pi/2);
link5 = Link('d',0,'a',0,'alpha',pi/2);
link6 = Link('d',135,'a',0,'alpha',0);
Tool = [eye(3),[0,0,100]';
        zeros(1,3),1];
robot_with_tool = SerialLink([link1, link2, link3, link4, link5, link6],'tool',Tool);
robot_without_tool = SerialLink([link1, link2, link3, link4, link5, link6]);
% robot_with_tool.plot(zeros(1,6));
Object_T = [eye(3),[1000,0,500]';
            zeros(1,3),1];
[Ts,p_measure] = gen_eye_calibration_sim(Object_T, 170, 2, 4);
qs = robot_with_tool.ikine(Ts);

Ts_without_tool= robot_without_tool.fkine(qs).double();
% for i=1:size(qs,1)
%     robot.plot(qs(i,:));
%     pause(2);
% end

hand_eye_calibration(Ts_without_tool, p_measure);
ans