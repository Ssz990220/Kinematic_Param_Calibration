function [p_c1s, p_c2s, L] = solve_common_normal(A,p1, B, p2)
%SOLVE_COMMON_NORMAL Summary of this function goes here
%   Detailed explanation goes here

a = [1,-A*B';A*B',-1];
if A==B
    sol = [0 0];
else
    sol = a\[p2*A'-p1*A';p2*B'-p1*B'];
end
l1s = sol(1);
l2s = sol(2);
p_c1s = p1 + l1s*A;
p_c2s = p2 + l2s*B;
L = norm(p_c1s - p_c2s);

end

