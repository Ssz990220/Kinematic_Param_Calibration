function H = hand_eye_calibration(Ts,p_measure, init, display_mode)
    %EYE_CALIBRATION Hand-eye parameter Calibration
    % Reference: A Self-Calibration Method for Robotic Measurement System
    % The return matrix H is an SE(3)
    % param: robot is a robot class that could execute motion and solve fkine &
    % ikine
    % param: n_samples descirbes how many samples we take to solve the
    % optimization problem. n_samples should be larger than 3 the make sure
    % there are enought equations to solve all the unknowns (6 unknowns).

    % Ts is last joint of manipulator in SE3, (4,4,n_samples)
    % p_measure is 1 by 3 coordinate indicates the relative position of the
    % object in the camera frame
    % H is in form:[n,o,a,p;0,0,0,0]
    % where n, o, a, p are all 3*1 vector,
    % with constraints ||a||=1, ||n|| = 1, ||o||=1
    %                               a'*n=0, n'*o=0, o'*a=0;
    if nargin < 4
        mode = 'none';
    else
        mode = display_mode;
    end
    n_sample = size(p_measure, 2);
    % Optimize Variable
    n  = optimvar('n',3,1);
    o  = optimvar('o',3,1);
    a  = optimvar('a',3,1);
    p_H = optimvar('p',3,1);
    n0 = init(1:3,1);
    o0 = init(1:3,2);
    a0 = init(1:3,3);
    p_H0 = init(1:3,4);
    x0 = struct('n',n0,'o',o0,'a',a0','p',p_H0);
    % constraints
%     constraint = optimconstr(6);
    constraint(1) = n'*n ==1;
    constraint(2) = a'*a ==1;
    constraint(3) = o'*o ==1;
    constraint(4) = a'*n ==0;
    constraint(5) = n'*o ==0;
    constraint(6) = o'*a ==0;

    R_H = [n o a];
    P_R = optimexpr(3,n_sample);
    for i=1:n_sample
        T_N = Ts(:,:,i);
        P = p_measure(:,i);
        R_N = T_N(1:3,1:3);
        P_N = T_N(1:3,4);
        P_R(:,i)=R_N*R_H*P+R_N*p_H + P_N;
    end
    
    E = optimexpr(1);
    for i=1:n_sample
        for j=(i+1):n_sample
            E = E + (P_R(:,i)-P_R(:,j))'*(P_R(:,i)-P_R(:,j));
        end
    end
    E = E/n_sample;
    
    prob = optimproblem('Objective', E,'Constraints',constraint);
    options = optimoptions(prob);
    options.MaxFunctionEvaluations = 10000;
    options.OptimalityTolerance = 1e-12;
    options.StepTolerance = 1e-32;
    options.Display = mode;
%     tic;
    sol = solve(prob,x0,'Options',options);
%     toc;
    H = [sol.n, sol.o, sol.a, sol.p;0,0,0,1];
end

