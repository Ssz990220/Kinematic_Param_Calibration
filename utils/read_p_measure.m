function p_measures = read_p_measure(filename)
%READ_REAL_MEASURE_DATA Summary of this function goes here
%   Detailed explanation goes here
file = fopen(filename,'r');
formatSpec = '[%f, %f, %f]\n';
p_measures = fscanf(file,formatSpec,[3, Inf]);
fclose(file);
end

