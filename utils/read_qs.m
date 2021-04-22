function angle_list= read_qs(filename)
%READ_QS Summary of this function goes here
%   Detailed explanation goes here
file = fopen(filename,'r');
formatSpec = '%f %f %f %f %f %f %f\n';
qs = fscanf(file,formatSpec,[6, Inf]);
qs = qs';
angle_list = qs/180*pi;
fclose(file);
end

