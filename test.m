load('abb_4600_param_poe.mat');
T_tool = eye(4);
g_st0 = [eye(3),[1270,0,1570]';
    zeros(1,3),1];
robot = my_poe_robot(z,q,g_st0,T_tool);
robot.fkine([0,0.2,0.2,0.2,0,0])