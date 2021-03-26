function [p_output] = ZigZag(P,row,column)
length = size(P,2);
% scan path
n = row;
m = column;
p_output = zeros(n*m,length);
count = 1;
signal = 1;
for i = 1:n
    for j = 1:m
        p_output(count,:) = P(signal*j+(i-1+(1-signal)/2)*m+(1-signal)/2,:);
        count = count + 1;
    end
    signal = -signal;
end
end