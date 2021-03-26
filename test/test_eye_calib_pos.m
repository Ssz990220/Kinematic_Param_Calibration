addpath(genpath('../'));
Object_T = [eye(3),[1300,0,1100]';
            zeros(1,3),1];
r = 200;

%% Test measure pose generation
[Ts,p_measure] = gen_eye_calibration_pos(Object_T,10,2,4,15,1);
Ts_plot = cat(3,Object_T,Ts);
p_measure_plot = [[0,0,0]',p_measure];
view_measure_pose(Ts,p_measure,1); % if all tilt arror point to the same point, it works


%% Validate on robot
[Ts,p_measure] = gen_eye_calibration_pos(Object_T,0,2,4,15,1);
robot = my_new_dh_robot();      % using default tool
% robot.plot(zeros([1,6]));
qs = robot.ikine(Ts);
robot_view_generate_pose(robot, qs);
robot.fkine(qs(1,:)).double() - robot.fkine(qs(8,:)).double()   % if (1:3, 4) of the result is [0 0 0]', it works as expected