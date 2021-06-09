function [err,err2,distance_matrix] = check_error(pos,real) 
[best_R,err,err2]=ICP_bestR(pos,real);
sample_size = size(pos,2);
raw_hole_data = pos;
distance_matrix = zeros(sample_size);
A=best_R\real;
for i=1:sample_size
    for j=1:sample_size
        distance_matrix(i,j)=norm(raw_hole_data(:,j)-raw_hole_data(:,i))-norm(A(:,j)-A(:,i));
        distance_matrix(i,j)=distance_matrix(i,j)/(norm(raw_hole_data(:,j)-raw_hole_data(:,i)));
    end
end
end