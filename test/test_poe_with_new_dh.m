addpath(genpath('..\'));

file = fopen('.\experiment_0324\2125_hand_eye_calibration\qs5.txt','r');
formatSpec = '%f %f %f %f %f %f\n';
qs = fscanf(file,formatSpec,[6, Inf]);
qs = qs/180*pi;
qs(5,:) = -qs(5,:);
qs = qs';
robot = my_new_dh_robot(eye(4));
robot_poe = my_poe_robot(eye(4));
error = 0;
for i  = 1:size(qs,1)
    Ts_dh = robot.fkine(qs(i,:)).double();
    Ts_poe = robot_poe.fkine(qs(i,:));
    error = error + norm(Ts_dh - Ts_poe);
end
error/size(qs,1)