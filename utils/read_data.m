function [qs,Ts,p_measure] = read_data(filename)
%READ_REAL_MEASURE_DATA Summary of this function goes here
%   Detailed explanation goes here
file = fopen(filename,'r');
formatSpec = '%d[%f,%f,%f][%f,%f,%f,%f]\n[%f,%f,%f,%f,%f,%f]\n[%f, %f, %f]\n';
data = fscanf(file,formatSpec,[17, Inf]);
fclose(file);
p_measure = data(15:17,:);
Ts = convert_real_robot_pos(data(2:8,:)');
qs = data(9:14,:);
qs = qs';
qs = qs/180*pi;
end