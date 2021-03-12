classdef my_poe_robot
    %MY_POE_ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        poe_omega;
        poe_q;
        n_dof;
        links;
        T_tool;
        g_st0;
    end
    
    methods
        function obj = my_poe_robot(poe_omega, poe_q, g_st0, T_tool)
            %MY_POE_ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            obj.poe_omega = poe_omega;  %  the axis that the joint is rotate about, in 3 by 1
            obj.poe_q = poe_q;      % a point on the axis, or v in prismatic case, in 3 by 1
            obj.n_dof = size(poe_omega,2);
%             obj.links = cell(obj.n_dof,1);
            obj.links = zeros(6,obj.n_dof); % the robot is represented in n_dof twists
            obj.g_st0 = g_st0;      % inital tool transformation matrix SE3
            for i = 1:obj.n_dof
%                 obj.links{i} = Twist('R',obj.poe_omega(:,i),obj.poe_q(:,i));
                v = -cross(obj.poe_omega(:,i),obj.poe_q(:,i));
                obj.links(:,i) = [v;obj.poe_omega(:,i)];
            end
            obj.T_tool = T_tool;
        end
        
        function T = fkine(obj,pose)
            %fkine Summary of this method goes here
            %   Detailed explanation goes here
            % TODO: How to calculate forward kinematics is norm(omega) ~= 1
            T = eye(4);
            for i=1:obj.n_dof
%                 T = T*obj.links{i}.exp(pose(i)).double();
                T = T*se3exp(obj.links(:,i),pose(i));
            end
            T = T*obj.g_st0*obj.T_tool;
        end
        
        function Jacob = get_J(obj, pose)
            current_T = eye(4);
            Jacob = zeros(6, 7*obj.n_dof + 6);
            for i = 1:obj.n_dof + 1
                Ad = Ad_X(current_T);
                if i~= obj.n_dof + 1
                    current_T = current_T * se3exp(obj.links(:,i),pose(i));      %update T
                end
                omega_att = norm(obj.poe_omega(i));                         %attitude of omega
                theta = omega_att * pose(i);
                Omega = [skew(obj.poe_omega(i)), zeros(3,3);
                        skew(obj.poe_a(i)), skew(obj.poe_omega(i))];
                    
                A = q(i) * eye(6) + (4 - theta*sin(theta)-4*cos(theta))/(2*omega_att^2) * Omega...
                    + (4*theta-5*sin(theta)+theta*cos(theta))/(2*omega_att^3)*Omega^2 ...
                    + (2 - theta*sin(theta) - 2*cos(theta))/(2*omega_att^4)*Omega^3 ...
                    +(2*theta -3*sin(theta) + theta*cos(theta))/(2*omega_att^5)*Omega^4;
                Jacob(:,7*(i-1)+1:7*(i-1)+7) = Ad * [A,[obj.poe_omega(i);obj.poe_a(i)]];
            end
        end
    end
        
%         function q = ikine(obj, target)
%             % target is represented in SE3
%             % !!!!!!!!!!!!!!!!!!!!!!
%             % this ikine function is specifically for ABB 4600-60/2.05
%             inv_g_st = inv_SE3(target);
%             g = target*inv_g_st;
%             q(1) = atan2(g(2,4),g(1,4));
%             % Use geometric method to solve q2 and q3
%             l = sqrt(g(2,4)^2 + g(1,4)^2);
%             dx = l-175;
%             dy = g(3,4)-495;
%             dl = sqrt(dx^2+dy^2);
%             l1 =900;  l2 = sqrt(175^2+960^2);
%             theta2 = acos((l1^2 + l2^2 - dl^2)/2*l1*l2);
%             alpha = atan2(175, 960);
%             if theta2 < 0
%                 q(3) = pi + theta2 - pi/2 - alpha;
%             else
%                 q(3) = pi/2 + alpha - theta2;
%             end
%             beta = atan2(dy, dx);
%             gamma = asin(l2*sin(abs(theta2))/dl);
%             q(2) = beta + gamma - pi/2;
%             
%             % solve q456
%             e1_3 = se3exp(obj.links(:,1),q(1))*se3exp(obj.links(:,2),q(2))*se3exp(obj.links(:,3),q(3));
%             inv_e1_3 = inv_SE3(e1_3);
%             
%         end
end

function SE3 = se3exp(twist, theta)
    omega = twist(4:6);
    SO3 = so3exp((omega),theta);
    sk_omega = skew(omega);
    omega_att = norm(omega);
    A = eye(3) + (1-cos(omega_att*theta))/omega_att^2*sk_omega + (omega_att - sin(omega_att*theta))/omega_att^3*sk_omega^2;
%     SE3 = [SO3, (eye(3)-SO3)*cross(twist(4:6),twist(1:3))+twist(4:6)*twist(4:6)'*twist(1:3)*theta;
%         zeros(1,3),1];
    SE3 = [SO3,A*twist(1:3);
        zeros(1,3),1];
end

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

% function SE3_inv = inv_SE3(target)
% SE3_inv = [target(1:3,1:3)',-target(1:3,1:3)'*target(1:3,4);
%                         zeros(1,3),1];
% end

function Ad = Ad_X(SE3)
% Convert SE3 matrix to a 6 by 6 adjoint matrix
Theta = SE3(1:3, 1:3);
b = SE3(1:3, 4);
Ad = [Theta, zeros(3,3);
    skew(b)*Theta, Theta];
end