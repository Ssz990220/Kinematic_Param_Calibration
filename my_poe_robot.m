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
        g_st_poe;
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
            g_st_se3 = MatrixLog6(g_st0);
%             g_st_se3 = log_SE3(g_st0);
            obj.g_st_poe = [un_skew(g_st_se3(1:3,1:3));g_st_se3(1:3,4)];
            for i = 1:obj.n_dof
%                 obj.links{i} = Twist('R',obj.poe_omega(:,i),obj.poe_q(:,i));
                v = -cross(obj.poe_omega(:,i),obj.poe_q(:,i));
                obj.links(:,i) = [obj.poe_omega(:,i);v];        % [omega, v]'
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
%             T = T*se3exp(obj.g_st_poe,1)*obj.T_tool;
            T = T * obj.g_st0 * obj.T_tool;
        end
        
        function Jacob = get_J(obj, pose)
            current_T = eye(4);
            Jacob = zeros(6, 7*obj.n_dof + 6);
            for i = 1:obj.n_dof + 1
                Ad = Ad_X(current_T);
                if i~= obj.n_dof + 1
                    current_T = current_T * se3exp(obj.links(:,i),pose(i));      %update T
                    omega_att = norm(obj.poe_omega(:,i));                         %attitude of omega
                    theta = omega_att * pose(i);
                    Omega = [skew(obj.poe_omega(:,i)), zeros(3,3);
                        skew(obj.poe_q(:,i)), skew(obj.poe_omega(:,i))];
                    A = pose(i) * eye(6) + (4 - theta*sin(theta)-4*cos(theta))/(2*omega_att^2) * Omega...
                        + (4*theta-5*sin(theta)+theta*cos(theta))/(2*omega_att^3)*Omega^2 ...
                        + (2 - theta*sin(theta) - 2*cos(theta))/(2*omega_att^4)*Omega^3 ...
                        +(2*theta -3*sin(theta) + theta*cos(theta))/(2*omega_att^5)*Omega^4;
                    Jacob(:,7*(i-1)+1:7*(i-1)+7) = Ad * [A,[obj.poe_omega(:,i);obj.poe_q(:,i)]];
                else
                    omega_att = norm(obj.g_st_poe(4:6));
                    Omega = [skew(obj.g_st_poe(4:6)),zeros(3,3);
                        skew(obj.g_st_poe(1:3)), skew(obj.g_st_poe(4:6))];
                    A = eye(6) + (4 - theta*sin(theta)-4*cos(theta))/(2*omega_att^2) * Omega...
                        + (4*theta-5*sin(theta)+theta*cos(theta))/(2*omega_att^3)*Omega^2 ...
                        + (2 - theta*sin(theta) - 2*cos(theta))/(2*omega_att^4)*Omega^3 ...
                        +(2*theta -3*sin(theta) + theta*cos(theta))/(2*omega_att^5)*Omega^4;
                    Jacob(:,7*obj.n_dof+1:7*obj.n_dof+6) = Ad * A;
                end
            end
        end
        
        function obj = update_poe(obj,delta_poe)
            delta_poe_kine = zeros(size(obj.links));
            for i = 1:obj.n_dof
               delta_poe_kine(:,i) = delta_poe(7*(i-1)+1:7*(i-1)+6); 
            end
            delta_poe_st  = delta_poe(7*obj.n_dof+1:7*obj.n_dof+6);
            obj.links = obj.links + delta_poe_kine;
            obj.g_st_poe = obj.g_st_poe + delta_poe_st;
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
    omega = twist(1:3);
    SO3 = so3exp((omega),theta);
    sk_omega = skew(omega);
    omega_att = norm(omega);
    if omega_att == 0
        SE3 = [eye(3),twist(4:6)*theta;
                zeros(1,3),1];
    else
        A = eye(3) + (1-cos(omega_att*theta))/omega_att^2*sk_omega + (omega_att - sin(omega_att*theta))/omega_att^3*sk_omega^2;
        %     SE3 = [SO3, (eye(3)-SO3)*cross(twist(4:6),twist(1:3))+twist(4:6)*twist(4:6)'*twist(1:3)*theta;
    %         zeros(1,3),1];
        SE3 = [SO3,A*twist(4:6);
            zeros(1,3),1];
    end
end

function SO3 = so3exp(omega, theta)
    sk_omega = skew(omega);
    omega_att = norm(omega);
    if omega_att ~= 0
        SO3 = eye(3)+sk_omega*sin(omega_att * theta)/omega_att+sk_omega^2*(1-cos(omega_att * theta))/omega_att^2;
    else
        SO3 = eye(3);
    end
%     SO3 = expm(skew(omega)*theta);
end

function sk_v = skew(vector)
    sk_v = [0 -vector(3), vector(2);
            vector(3),0,-vector(1);
            -vector(2),vector(1),0];
end

function so3_coor = un_skew(so3)
    so3_coor = [-so3(2,3),so3(1,3),-so3(1,2)]';
end

% function SE3_inv = inv_SE3(target)
% SE3_inv = [target(1:3,1:3)',-target(1:3,1:3)'*target(1:3,4);
%                         zeros(1,3),1];
% end
% function se3 = log_SE3(SE3)
%     theta = SE3(1:3,1:3);
%     b = SE3(1:3,4);
%     phi = acos((trace(theta)-1)/2);
%     sk_omega = phi/(2*sin(phi))*(theta-theta');
%     omega = un_skew(sk_omega);
%     norm_omega = norm(omega);
%     if norm_omega == 0
%         se3 = [sk_omega, b;
%                 zeros(1,4)];
%     else
%         A_inv = eye(3) - 1/2*sk_omega + (2*sin(norm_omega) - norm_omega*(1+cos(norm_omega)))/(2*norm_omega^2*sin(norm_omega));
%         se3 = [sk_omega,A_inv * b;
%         zeros(1,4)];
%     end
% end

function Ad = Ad_X(SE3)
% Convert SE3 matrix to a 6 by 6 adjoint matrix
Theta = SE3(1:3, 1:3);
b = SE3(1:3, 4);
Ad = [Theta, zeros(3,3);
    skew(b)*Theta, Theta];
end