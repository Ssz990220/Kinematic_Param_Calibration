addpath('../');
addpath('../mr');
Object_T = [eye(3),[1300,0,1100]';
            zeros(1,3),1];
r = 200;
[Ts,p_measure] = gen_eye_calibration_pos(Object_T,0,2,4,15,0);
figure

Ts = [Object_T,Ts];
p_measure = [[0,0,0]',p_measure];
view_measure_pose(Ts,p_measure,1);