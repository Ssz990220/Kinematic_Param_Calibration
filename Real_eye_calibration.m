% clear;
% clc;
% link1 = Link('d',495,'a',175,'alpha',pi/2);
% link2 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
% link3 = Link('d',0,'a',175,'alpha',pi/2);
% link4 = Link('d',960,'a',0,'alpha',-pi/2);
% link5 = Link('d',0,'a',0,'alpha',pi/2);
% link6 = Link('d',135,'a',0,'alpha',0);
% robot_without_tool = SerialLink([link1, link2, link3, link4, link5, link6]);
% % robot_with_tool.plot(zeros(1,6));
% qs = load('qs5.txt');
% qs = qs(1:4,:);

%% load measure position
% p_measure = [-2.25387, 41.1774, -2.36082;
%             6.31666, 41.9971, -3.89512;
%             -12.1279, 23.9648, 11.5311;
%             11.3552, 26.348, 10.7816]';   
        
%%
% Ts_without_tool= robot_without_tool.fkine(qs).double();
filename = './experiment_0324/2125_hand_eye_calibration/endT_data.txt';
file = fopen(filename,'r');
formatSpec = '[%f,%f,%f][%f,%f,%f,%f]\n';
last_link_pos = fscanf(file,formatSpec,[7, Inf]);
fclose(file);
last_link_pos = last_link_pos';
                        
Ts = convert_real_robot_pos(last_link_pos);

% Ts_valid_index = [18,19,20,21,22,23,26,27,28,29,30,31,2,3,4,5,6,7,10,11,12,13,14,15];
% Ts_valid = Ts(:,:,Ts_valid_index);
p_measure = [8.10051, 0.755453, 3.87203;
5.05026, -4.90213, 6.82202;
1.36852, -1.74397, -5.99634;
1.39138, -1.73278, -5.9723;
-2.09751, -9.1223, 6.44667;
-0.961975, -8.42116, 6.71552;
8.50662, 0.402165, 0.487261;
1.12939, 5.76839, 3.31967;
-2.03222, -0.78246, -1.64654;
9.88228, 4.39013, 3.67442;
-1.41321, 8.18005, -8.83883;
3.01317, -4.02275, 8.21561;
2.08095, -5.01704, 4.58074;
-1.33683, 0.500471, -8.73315;
10.6069, -7.12068, 5.8019;
3.8862, -1.62116, 4.45835;
6.73956, 3.15621, 8.00679;
-2.49731, 9.27199, 8.87429;
-2.06716, -9.3247, -9.94785;
12.4618, -4.71988, -9.57101;
5.57377, -3.75484, 0.803766;
-4.54781, -5.06984, -3.29962;
-3.29673, -4.83072, -7.26155;
9.99682, -9.62399, -8.29242;
5.36101, -0.873521, 0.0862045;
-0.205442, -0.935163, 7.52743;]';
n_points_used = 26;
Ts = Ts(:,:,1:n_points_used);
p_measure = p_measure(:,1:n_points_used);
R_ = [-1,0,0;0,1,0;0,0,-1]';
init = [R_,[0,0,370]';
        zeros(1,3),1];
Tool_T = hand_eye_calibration(Ts, p_measure,init)


% Tool = [eye(3),[0,0,270]';
%         zeros(1,3),1];
% robot_with_tool = SerialLink([link1, link2, link3, link4, link5, link6],'tool',Tool_T);
% pos_eff = robot_with_tool.fkine(qs(1,:)).double();
% ball_pos = pos_eff * [eye(3),qs(1,:)';zeros(1,3),1];
% ball_pos = ball_pos(1:3,4)
