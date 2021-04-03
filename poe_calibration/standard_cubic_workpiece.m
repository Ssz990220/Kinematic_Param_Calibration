classdef standard_cubic_workpiece
    %STANDARD_CUBIC_WORKPIECE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Ts_origin;
        type;
        Ts_face1;
        Ts_face2;
        Ts_face3;
        n_holes_each_line;
        dis_holes;
        edge_length;
        n_holes;
        n_holes_each_face;
    end
    
    methods
        function obj = standard_cubic_workpiece(Ts_origin, type, n_holes_each_line, dis_holes, edge_length)
            %STANDARD_CUBIC_WORKPIECE Construct an instance of this class
            %   Type 1 cubic has the x-y plane at z=edge_length surface
            %   Type 2 cubic has the x-y plane at z = 0 surface;
            obj.Ts_origin = Ts_origin;
            obj.type = type;
            obj.n_holes_each_line = n_holes_each_line;
            obj.dis_holes = dis_holes;
            obj.edge_length = edge_length;
            holes_coor = linspace(1,n_holes_each_line,n_holes_each_line) * dis_holes;
            obj.Ts_face1 = repmat(eye(4),1,1, n_holes_each_line^2);
            obj.Ts_face2 = repmat(eye(4),1,1, n_holes_each_line^2);
            obj.Ts_face3 = repmat(eye(4),1,1, n_holes_each_line^2);
            obj.n_holes_each_face = n_holes_each_line ^ 2;
            obj.n_holes = obj.n_holes_each_face * 3;
            
            %% Face X-Y
            if type == 1
                z = edge_length;
            elseif type == 2
                z = 0;
            end
            for i = 1:n_holes_each_line
                x = holes_coor(i);
                for j = 1:n_holes_each_line
                    y = holes_coor(j);
                    obj.Ts_face1(1:3,1:3,(i-1)*n_holes_each_line+j) = rotz(45);
                    obj.Ts_face1(1:3,4,(i-1)*n_holes_each_line+j) = [x y z]';
                    obj.Ts_face1(:,:,(i-1)*n_holes_each_line+j) = obj.Ts_origin * obj.Ts_face1(:,:,(i-1)*n_holes_each_line+j);
                end
            end
            %% Face X-Z
            y = 0;
            for i = 1:n_holes_each_line
                x = holes_coor(i);
                for j = 1:n_holes_each_line
                    z = holes_coor(j);
                    obj.Ts_face2(1:3,1:3,(i-1)*n_holes_each_line+j) = rotz(90);
                    obj.Ts_face2(1:3,4,(i-1)*n_holes_each_line+j) = [x y z]';
                    obj.Ts_face2(:,:,(i-1)*n_holes_each_line+j) = obj.Ts_origin * obj.Ts_face2(:,:,(i-1)*n_holes_each_line+j);
                end
            end
            %% Face Y-Z
            x = 0;
            for i = 1:n_holes_each_line
                y = holes_coor(i);
                for j = 1:n_holes_each_line
                    z = holes_coor(j);
                    obj.Ts_face3(1:3,4,(i-1)*n_holes_each_line +j) = [x y z]';
                    obj.Ts_face3(:,:,(i-1)*n_holes_each_line+j) = obj.Ts_origin * obj.Ts_face3(:,:,(i-1)*n_holes_each_line+j);
                end
            end
        end
        function Ts = get_all_Ts(obj)
            Ts = zeros(4,4, obj.n_holes);
            Ts(:,:,1:obj.n_holes_each_face) = obj.Ts_face1;
            Ts(:,:,obj.n_holes_each_face+1:2*obj.n_holes_each_face) = obj.Ts_face2;
            Ts(:,:,2*obj.n_holes_each_face+1:3*obj.n_holes_each_face) = obj.Ts_face3;
        end
    end
end

