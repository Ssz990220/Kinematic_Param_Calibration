function [p_measures,Ts] = read_real_measure_data(filename)
%READ_REAL_MEASURE_DATA Summary of this function goes here
%   Detailed explanation goes here
file = fopen(filename,'r');
formatSpec = '%d[%f,%f,%f][%f,%f,%f,%f][%f, %f, %f]\n';
data = fscanf(file,formatSpec,[11, Inf]);
fclose(file);
p_measures = data(9:11,:);
Ts = convert_real_robot_pos(data(2:8,:)');
end

