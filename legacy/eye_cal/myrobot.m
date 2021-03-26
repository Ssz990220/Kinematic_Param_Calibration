classdef myrobot
    %MYROBOT is a robot class that inherit ur10 robot in robotic toolbox
    %(Peter Corke) and integrates matlab built in ROS interface.
    
    properties
       n_dof;
       p_robot;
       dh;
       joint_type;
    end
    
    methods
        function obj = myrobot(dh_init, joint_type)
            %MYROBOT Construct an instance of this class
            %   Detailed explanation goes here
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                    PARAMETERS                     %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % p_robot is a UR10 robot class from Prof. Peter Corke's
            % robotic toolbox
            %
            
            obj.p_robot = p_robot(dh_parameter);      
            % initiate a Peter Corke's robot with dh_init parameter
            obj.dh = dh_init;
            obj.joint_type = joint_type;
        end
        
        function T = fkine(obj, poses)
            % fkine: Forward kinematic of robot
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                            PARAMETERS                             %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % poses: robot joint displacement.
            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                OUTPUT                                   %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % T is a 4*4 SE(3) Matrix.
            %
%             T = obj.p_robot.fkine(poses);     # Peter Corke

        end
        
        function T = move(obj, target)
            %move: move to a target location represented in SE(3)
            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                   OUTPUT                                %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % T is a 4*4 SE(3) Matrix that shows the location and
            % orientation of end-effector in robot base frame
            %
            
        end
        
        function p = measure_ball(obj)
            % take a picture and measure the center of the calibration ball
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                                   OUTPUT                                %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %
            % P is a 1*3 vector that represent the ball location in camera
            % frame
            %
        end
        
        function As = get_As(obj, pose)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                            PARAMETERS                             %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % pose is in shape(num_of_poses, n_dof)
            %
            As = zeros(4,4,obj.n_dof,size(pose));
            for j=1:obj.n_dof
                As(:,:,j) = obj.p_robot.links(j).A(pose(j,2));
            end
        end
        
        function Us = get_Us(obj, pose)
            As = obj.get_As(pose);
            Us = zeros(4,4,obj.n_dof,size(pose,2));
            Us(:,:,size(pose,2))= eye(4);
            for j=obj.n_dof-1:-1:1
                Us(:,:,j) = As(:,:,j+1)*Us(:,:,j+1);
            end
        end
        
        function Js = get_jacobian(obj, poses)
            Js = zeros(6* size(poses,1), obj.n_dof*5);
            for i = 1:size(poses,1)
                pose = poses(i,:);
                Us = obj.get_Us(pose);
                for j = 1:obj.n_dof
                    
                end
            end
        end
        
    end
end

