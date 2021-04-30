classdef Cubes_array
    %CUBES_ARRAY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cubes;
        n_cubes;
        Ts_face1;
        Ts_face2;
        Ts_face3;
    end
    
    methods
        function obj = Cubes_array(cubes)
            %CUBES_ARRAY Construct an instance of this class
            %   Detailed explanation goes here
            obj.cubes = cubes;
            obj.n_cubes = size(cubes,2);
            obj.Ts_face1 = [];
            obj.Ts_face2 = [];
            obj.Ts_face3 = [];
            for i  = 1:obj.n_cubes
                obj.Ts_face1 = cat(3, obj.Ts_face1, cubes(i).Ts_face1);
            end
            for i  = 1:obj.n_cubes
                obj.Ts_face2 = cat(3, obj.Ts_face2, cubes(i).Ts_face2);
            end
            for i  = 1:obj.n_cubes
                obj.Ts_face3 = cat(3, obj.Ts_face3, cubes(i).Ts_face3);
            end
        end
        
        function Ts = get_all_Ts(obj)
            Ts =[];
            for i = 1:obj.n_cubes
                Ts_cur = obj.cubes(i).get_all_Ts();
                Ts = cat(3, Ts, Ts_cur);
            end
        end
    end
end

