function [best_R,err,err2]=ICP_bestR(raw,test)
num=size(raw,2);
H=(raw-sum(raw,2)/num)*(test-sum(test,2)/num)';
[U,~,V]=svd(H);d=sign(det(V*U'));
best_R=V*[1 0 0;0 1 0;0 0 d]*U';
% best_R=eye(3);
% best_R=[0 1 0;-1 0 0;0 0 1];
raw_data_in_part_center=(raw-sum(raw,2)/num);
measure_result_in_CMM_cordinate=(best_R\test-sum(best_R\test,2)/num);
err=abs(raw_data_in_part_center-measure_result_in_CMM_cordinate);
err=err.^2;err=sum(err,1)/3;err=err.^(1/2);err=sum(err)/num;    %平均每个点处的位置测量误差
err2=raw_data_in_part_center-measure_result_in_CMM_cordinate;
end