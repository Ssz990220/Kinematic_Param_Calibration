classdef my_poe_robot < handle
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
        angle_error;
        angle_shift;
    end
    
    methods
        function obj = my_poe_robot(T_tool, add_joint_shift, omega_shift_level, q_shift_level, add_base_shift, base_shift_omega, base_shift_q, add_angle_noise, angle_error_level, angle_error_decay)
            %MY_POE_ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            poe_omega = [0 0 1;
                    0 1 0;
                    0 1 0;
                    1 0 0;
                    0 1 0;
                    1 0 0]';
            poe_q = [0 0 0;
                175 0 495;
                175 0 1395;
                175 0 1570;
                1135 0 1570;
                1270 0 1570]';
            g_st0 = [0,0,1,1270;
                            0,1,0,0;
                            -1,0,0,1570;
                            0,0,0,1];
                        
            %% Process input argument
            if nargin == 0              % default parameter
                R_ = [-1,0,0;0,1,0;0,0,-1]';
                T_tool= [R_,[0,0,370]';
                        zeros(1,3),1];
                add_joint_shift = false;
                omega_shift_level = 0;
                q_shift_level = 0;
                add_base_shift = false;
                base_shift_omega = 0;
                base_shift_q = 0;
                add_angle_noise = false;
                angle_error_level = 0;
                angle_error_decay = 0;
            elseif nargin == 1
                add_joint_shift = false;
                omega_shift_level = 0;
                q_shift_level = 0;
                add_base_shift = false;
                base_shift_omega = 0;
                base_shift_q = 0;
                add_angle_noise = false;
                angle_error_level = 0;
                angle_error_decay = 0;
            elseif nargin == 7
                add_angle_noise = false;
                angle_error_level = 0;
                angle_error_decay = 0;
            end
            
            
            %% Initialize
            if add_joint_shift
                omega_noise = normrnd(0,omega_shift_level,size(poe_omega));
                q_noise = normrnd(0,q_shift_level,size(poe_q));
                omega_fake = poe_omega + omega_noise;
                for i = 1:6
                    omega_fake(:,i) = omega_fake(:,i)/norm(omega_fake(:,i));
                end
                poe_q = poe_q + q_noise;
                poe_omega = omega_fake;
            end
            obj.poe_omega = poe_omega;  %  the axis that the joint is rotate about, in 3 by 1
            obj.poe_q = poe_q;      % a point on the axis, or v in prismatic case, in 3 by 1
            obj.n_dof = size(poe_omega,2);
            obj.links = zeros(6,obj.n_dof); % the robot is represented in n_dof twists
            obj.g_st0 = g_st0;      % inital tool transformation matrix SE3
            g_st_se3 = MatrixLog6(g_st0);
            obj.g_st_poe = [un_skew(g_st_se3(1:3,1:3));g_st_se3(1:3,4)];
            for i = 1:obj.n_dof
                obj.poe_omega(:,i) = poe_omega(:,i)/norm(poe_omega(:,i));
            end
            for i = 1:obj.n_dof
                v = -cross(obj.poe_omega(:,i),obj.poe_q(:,i));
                obj.links(:,i) = [obj.poe_omega(:,i);v];        % [omega, v]'
            end
            if add_angle_noise
                angle_error = (rand([1,6]) - 0.5) * 2 * angle_error_level;
                weights = linspace(-obj.n_dof+1,0,obj.n_dof);
                weights = angle_error_decay.^weights;
                obj.angle_error = angle_error .* weights;
            else
                obj.angle_error = zeros(1,obj.n_dof);
            end
            obj.angle_shift = zeros(1,obj.n_dof);
            obj.T_tool = T_tool;
            if add_base_shift
                % omega in g_st need not to be normalized
                g_st_poe_omega_noise = normrnd(0,base_shift_omega, [3,1]);
                g_st_poe_q_noise = normrnd(0,base_shift_q,[3,1]);
                g_st_omega_fake = obj.g_st_poe(1:3) + g_st_poe_omega_noise;
                g_st_poe_q_fake = obj.g_st_poe(4:6) + g_st_poe_q_noise;
                obj.g_st_poe = [g_st_omega_fake; g_st_poe_q_fake];
            end
        end
        
        %% Other Functions
        function T = fkine(obj,pose)
            %fkine Summary of this method goes here
            %   Forward kinematics for a POE robot
                T = eye(4);
                pose = pose + obj.angle_error + obj.angle_shift;
                for j=1:obj.n_dof
                        T = T*MatrixExp6(VecTose3(obj.links(:,j)*pose(j)));
                end
                T = T * MatrixExp6(VecTose3(obj.g_st_poe)) * obj.T_tool;
        end
        
        function Jacob = get_J(obj, pose, type)
            % type 1: only joint parameter is calibrated
            % type 2: joint parameter and g_st0 are all calibrated
            % type 3: joint parameters, g_st0 and joint offset are all calibrated.
            current_T = eye(4);
            if type == 1
                Jacob = zeros(6, 6*obj.n_dof);
            elseif type == 2
                Jacob = zeros(6, 6*obj.n_dof+6);
            elseif type == 3
                Jacob = zeros(6,7*obj.n_dof);
            end
            for i = 1:obj.n_dof
                Ad = Ad_X(current_T);
                current_T = current_T * se3exp(obj.links(:,i),pose(i));      %update T
                omega_att = norm(obj.links(1:3,i));                         %attitude of omega
                theta = omega_att * pose(i);
                Omega = [skew(obj.links(1:3,i)), zeros(3,3);                % the big omega is way different from omega
                    skew(obj.links(4:6,i)), skew(obj.links(1:3,i))];
                A = pose(i) * eye(6) + (4 - theta*sin(theta)-4*cos(theta))/(2*omega_att^2) * Omega...
                    + (4*theta-5*sin(theta)+theta*cos(theta))/(2*omega_att^3)*Omega^2 ...
                    + (2 - theta*sin(theta) - 2*cos(theta))/(2*omega_att^4)*Omega^3 ...
                    +(2*theta -3*sin(theta) + theta*cos(theta))/(2*omega_att^5)*Omega^4;
                if (type == 1) || (type == 2)
                    Jacob(:,6*(i-1)+1:6*i) = Ad * A;
                elseif type == 3
                    Jacob(:,7*(i-1)+1:7*i) = [Ad * A, obj.links(:,i)];
                end
            end
            if (type == 2)
                Ad = Ad_X(current_T);
                omega_att = norm(obj.g_st_poe(1:3));
                theta = omega_att;
                Omega = [skew(obj.g_st_poe(1:3)),zeros(3,3);
                    skew(obj.g_st_poe(4:6)), skew(obj.g_st_poe(1:3))];
                A = eye(6) + (4 - theta*sin(theta)-4*cos(theta))/(2*omega_att^2) * Omega...
                    + (4*theta-5*sin(theta)+theta*cos(theta))/(2*omega_att^3)*Omega^2 ...
                    + (2 - theta*sin(theta) - 2*cos(theta))/(2*omega_att^4)*Omega^3 ...
                    +(2*theta -3*sin(theta) + theta*cos(theta))/(2*omega_att^5)*Omega^4;
               Jacob(:,end-5:end) = Ad * A;
            end
        end
        
        function obj = update_poe(obj,delta_poe, type)
            % type 1: only joint parameter is calibrated
            % type 2: joint parameter and g_st0 are all calibrated
            delta_poe_kine = zeros(size(obj.links));
            for i = 1:obj.n_dof
                if (type == 1) || (type == 2)
                    delta_poe_kine(:,i) = delta_poe(6*(i-1)+1:6*i); 
                elseif type==3
                    delta_poe_kine(:,i) = delta_poe(7*(i-1)+1:7*(i-1) + 6); 
                end
            end
            cur_links = obj.links + delta_poe_kine;
            for i = 1:obj.n_dof
                cur_links(1:3,i) = cur_links(1:3,i)/norm(cur_links(1:3,i));
            end
            for i = 1:obj.n_dof
                cur_links(4:6,i) = cur_links(4:6,i) - (cur_links(1:3,i)'*cur_links(4:6,i))/(cur_links(1:3,i)'*cur_links(1:3,i))*cur_links(1:3,i);
            end
            obj.links = cur_links;  
             if type == 2
                delta_g_st0 = delta_poe(6*obj.n_dof+1:6*obj.n_dof+6);
                cur_g_st = obj.g_st_poe + delta_g_st0;
                obj.g_st_poe = cur_g_st;
             end
             if type == 3
                 angle_shift_update  = zeros(1,obj.n_dof);
                 for i = 1:obj.n_dof
                     angle_shift_update(i) = delta_poe(7*i);
                 end
                 obj.angle_shift  = obj.angle_shift - angle_shift_update;
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
    omega = twist(1:3);
    SO3 = so3exp((omega),theta);
    sk_omega = skew(omega);
    omega_att = norm(omega);
    if omega_att == 0
        SE3 = [eye(3),twist(4:6)*theta;
                zeros(1,3),1];
    else
        A = eye(3) + (1-cos(omega_att*theta))/omega_att^2*sk_omega + (omega_att - sin(omega_att*theta))/omega_att^3*sk_omega^2;
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
end

function sk_v = skew(vector)
    sk_v = [0 -vector(3), vector(2);
            vector(3),0,-vector(1);
            -vector(2),vector(1),0];
end

function so3_coor = un_skew(so3)
    so3_coor = [-so3(2,3),so3(1,3),-so3(1,2)]';
end

function Ad = Ad_X(SE3)
% Convert SE3 matrix to a 6 by 6 adjoint matrix
Theta = SE3(1:3, 1:3);
b = SE3(1:3, 4);
Ad = [Theta, zeros(3,3);
    skew(b)*Theta, Theta];
end