function [r] = cha(a,b)
hight_a=[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
r=hight_a*b;
end