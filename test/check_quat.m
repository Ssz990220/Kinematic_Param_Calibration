q = [0.50288 0.01181 0.86428 0.00000]; 
R =[2*q(1)^2-1+2*q(2)^2, 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(2)*q(4)+q(1)*q(3));
            2*(q(2)*q(3)+q(1)*q(4)), 2*q(1)^2-1+2*q(3)^2, 2*(q(3)*q(4)-q(1)*q(2));
            2*(q(2)*q(4)-q(1)*q(3)), 2*(q(3)*q(4)+q(1)*q(2)), 2*q(1)^2-1+2*q(4)^2]
        SO3_lj = rotx(178.64)*roty(60.37)*rotz(177.64)