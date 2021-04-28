function [qs,Ts] = read_ts_qs(filename)
%READ_REAL_MEASURE_DATA Summary of this function goes here
%   Detailed explanation goes here
file = fopen(filename,'r');
formatSpec = '[%f,%f,%f][%f,%f,%f,%f]\n[%f,%f,%f,%f,%f,%f]\n';
data = fscanf(file,formatSpec,[13, Inf]);
fclose(file);
qs = data(8:13,:)/180*pi;
qs = qs';
Ts = convert_real_robot_pos(data(1:7,:)');
end

