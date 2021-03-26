function Ts= read_real_robot_pos(filename)
%READ_REAL_ROBOT_POS Summary of this function goes here
%   Thsi function reads read robot last joint position in format [x y z q1
%   q2 q3 q4].
% filename = './experiment_0324/2125_hand_eye_calibration/endT_data.txt';
file = fopen(filename,'r');
formatSpec = '[%f,%f,%f][%f,%f,%f,%f]\n';
last_link_pos = fscanf(file,formatSpec,[7, Inf]);
fclose(file);
Ts = convert_real_robot_pos(last_link_pos');
end

