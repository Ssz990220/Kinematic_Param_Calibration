function p_measures= read_real_ball_measure(filename)
%READ_REAL_ROBOT_POS Summary of this function goes here
%   Thsi function reads read robot last joint position in format [x y z q1
%   q2 q3 q4].
% filename = './experiment_0324/2125_hand_eye_calibration/endT_data.txt';
file = fopen(filename,'r');
formatSpec = '%f, %f, %f\n';
p_measures = fscanf(file,formatSpec,[3, Inf]);
fclose(file);
end

