syms theta beta alpha a d
A = [cos(theta)*cos(beta)-sin(theta)*sin(alpha)*sin(beta)   -sin(theta)*cos(alpha)  cos(theta)*sin(beta)+sin(theta)*sin(alpha)*cos(beta)    a*cos(theta);
        sin(theta)*cos(beta)+cos(theta)*sin(alpha)*sin(beta)    cos(theta)*cos(alpha)   sin(theta)*sin(beta)-cos(theta)*sin(alpha)*cos(beta)    a*sin(theta);
        -cos(alpha)*sin(beta)                                                           sin(alpha)                      cos(alpha)*cos(beta)                                                        d;
        0                                                                                               0                                       0                                                                                     1];
A_theta = diff(A,theta);
A_beta = diff(A, beta);
A_alpha = diff(A, alpha);
A_a = diff(A,a);
A_d = diff(A,d);
Q_theta_left = [0 -1 0 0; 1 0 0 0;0 0 0 0; 0 0 0 0]; 
Q_theta = [0 cos(alpha)*cos(beta) sin(alpha) -a*sin(alpha)*sin(beta); cos(alpha)*cos(beta) 0 cos(alpha)*sin(beta) -a*cos(alpha); -sin(alpha) cos(alpha)*sin(beta) 0 a*sin(alpha)*cos(beta); 0 0 0 0]; % Checked
Q_beta = [0 0 1 0; 0 0 0 0; -1 0 0 0; 0 0 0 0]; % Checked
Q_alpha = [0 -sin(beta) 0 0;sin(beta) 0 -cos(beta) 0;0 cos(beta) 0 0; 0 0 0 0]; % Checked
Q_a = [zeros(4,3), [cos(beta);0;sin(beta);0]]; % Checked
Q_d = [0 0 0 -cos(alpha)*sin(beta); 0 0 0 sin(alpha); 0 0 0 cos(alpha)*cos(beta); 0 0 0 0]; % Checked
