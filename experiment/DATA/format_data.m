surfix = './experiment/experiment_0426/1448/'; 
filename = strcat(surfix,'p1.txt');
p_measure = read_p_measure(filename);
filename = strcat(surfix,'ts_qs1.txt');
file = fopen(filename,'r');
formatSpec = '[%f,%f,%f][%f,%f,%f,%f]\n[%f,%f,%f,%f,%f,%f]\n';
data = fscanf(file,formatSpec,[13, Inf]);
fclose(file);
str_output = '';
for i = 1:size(p_measure,2)
    string = sprintf('[%f,%f,%f][%f,%f,%f,%f]\n[%f,%f,%f,%f,%f,%f]\n[%f, %f, %f]\n',[data(:,i)',p_measure(:,i)']);
    str_output = strcat(str_output,'\n',num2str(i),string);
end
fid = fopen('str_out.txt','wt');
fprintf(fid, str_output);
fclose(fid);