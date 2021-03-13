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
load('abb_4600_param_poe.mat');
g_st0 = [0,0,1,1270;
        0,-1,0,0;
        1,0,0,1570;
        0,0,0,1];
robot_poe = my_poe_robot(z,q,g_st0,Tool);

