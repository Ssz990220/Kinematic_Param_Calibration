addpath('../');
addpath('../mr');
a = [1,0,0];
theta = 0.3;

so3exp(a*4, theta)
exp(skew(a*4*theta))
function SO3 = so3exp(omega, theta)
    sk_omega = skew(omega);
    omega_att = norm(omega);
    SO3 = eye(3)+sk_omega*sin(theta)/omega_att+sk_omega^2*(1-cos(theta))/omega_att^2;
end

function sk_v = skew(vector)
    sk_v = [0 -vector(3), vector(2);
            vector(3),0,-vector(1);
            -vector(2),vector(1),0];
end