function p_measures = read_p_measure(filename)
%READ_REAL_MEASURE_DATA Summary of this function goes here
%   Detailed explanation goes here
file = fopen(filename,'r');
formatSpec = '[%f, %f, %f]\n';
data = fscanf(file,formatSpec,[3, Inf]);
fclose(file);
p_measures = data(1:3,:);
end

