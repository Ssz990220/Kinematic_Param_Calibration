% This script generates kinematics parameters for ABB 4600-60/2.05
% industrial robot.

clear;
clc;
z = [0 0 1;
    0 -1 0;
    0 -1 0;
    1 0 0;
    0 -1 0;
    1 0 0]';
q = [0 0 0;
    175 0 495;
    175 0 1395;
    175 0 1570;
    1135 0 1570;
    1270 0 1570]';
z_noise = normrnd(0,0.01,size(z));
q_noise = normrnd(0,5,size(q));
z_fake = z + z_noise;
q_fake = q + q_noise;