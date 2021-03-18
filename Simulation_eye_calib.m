clear;
clc;
link1 = Link('d',495,'a',175,'alpha',pi/2);
link2 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
link3 = Link('d',0,'a',175,'alpha',pi/2);
link4 = Link('d',960,'a',0,'alpha',-pi/2);
link5 = Link('d',0,'a',0,'alpha',pi/2);
link6 = Link('d',135,'a',0,'alpha',0);

T_y = @(alpha_y)[cos(alpha_y),0,sin(alpha_y),0;
        0,1,0,0;
        -sin(alpha_y),0,cos(alpha_y),0;
        0,0,0,1];
T_z = @(alpha_z)[cos(alpha_z),-sin(alpha_z),0,0;
        sin(alpha_z),cos(alpha_z),0,0;
        0,0,1,0;
        0,0,0,1];
R = T_y(20/180*pi) * T_z(45/180*pi);
R = R(1:3,1:3);
Tool = [eye(3),[10,0,100]';
        zeros(1,3),1];
robot_with_tool = SerialLink([link1, link2, link3, link4, link5, link6],'tool',Tool);
robot_without_tool = SerialLink([link1, link2, link3, link4, link5, link6]);
% robot_with_tool.plot(zeros(1,6));
% ball_pos = load('ball_pos.mat');
Object_T = [eye(3),[1300,0,1100]';
            zeros(1,3),1];
[Ts,p_measure] = gen_eye_calibration_sim(Object_T, 10, 3, 12);
qs = robot_with_tool.ikine(Ts);
%%
% Generate joint angle for ABB
% qs_new = zeros(size(qs));
% for i = 1:8
%     qs_new((i-1)*2 + 1,:) = qs(i,:);
%     qs_new((i-1)*2 + 2,:) = qs(i+8,:);
% end

%%
% Solve
Ts_without_tool= robot_without_tool.fkine(qs).double();
% for i=1:size(qs,1)
%     robot_with_tool.plot(qs(i,:));
%     pause(2);
% end

T_tool = hand_eye_calibration(Ts_without_tool, p_measure)

%% Check results
% T_tool = [eye(3),[0,0,100]';zeros(1,3),1];
R_H = T_tool(1:3,1:3);
p_H = T_tool(1:3,4);
n_sample = size(p_measure, 2);
P_R = zeros(3,n_sample);
for i=1:n_sample
    T_N = Ts_without_tool(:,:,i);
    P = p_measure(:,i);
    R_N = T_N(1:3,1:3);
    P_N = T_N(1:3,4);
    P_R(:,i)=R_N*R_H*P+R_N*p_H + P_N;
end
E = 0;
for i=1:n_sample
    for j=(i+1):n_sample
        E = E + (P_R(:,i)-P_R(:,j))'*(P_R(:,i)-P_R(:,j));
    end
end
E