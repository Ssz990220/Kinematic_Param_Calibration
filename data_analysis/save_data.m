clear;close all;clc;
surfix = './../../gocator_pcl/src/pcl_pub/results/';
robot = {'raw','TP','POE','hybrid'};
obj = {'car_hor','car_ver','car_var','car_avg','cube'};
batch_size = [4,4,4,4,4];

for i = 1:4 % robot
    for j = 1:5 % obj
        Tool_T_path = strcat(surfix, 'Tool_T_',robot{i},'.mat');
        load(Tool_T_path, 'Tool_T');

        robot_poe = my_poe_robot(eye(4));
        if any(i == [3,4])
            x_path = strcat(surfix, 'x_',robot{i},'.mat');
            load(x_path, 'x');
            robot_poe.initialize(x);
        end
        
        real_path = strcat(surfix, '/real.mat');
        load(real_path, 'real');
        if j == 5
            real_path = strcat(surfix, '/real_cube.mat');
            load(real_path, 'real');
        end
        mask = [];
        count = 0;
        Ball_Pos = [];
        Ts_record = [];
        err = [];
        err2 = [];
        for number = 1:batch_size(j)
            if any(number == mask)
                continue
            end

            count = count+1;
            filename = strcat(surfix,obj{j},'/',num2str(number),'.txt');
            
            if any(i == 2)
                [~,Ts,p_measure] = read_data(filename);
            else
                [qs,~,p_measure] = read_data(filename);
                Ts = robot_poe.fkines(qs);
            end
            % record data
            [Ball_Pos{count},Ts_record{count}] = ball_pos(p_measure, Ts, Tool_T);
            [err{count},err2{count},distance_matrix] = check_error(Ball_Pos{count},real);
        end
        sample_size = size(Ball_Pos,2);

        filename = strcat(surfix,obj{j},'/Raw_Pos_',robot{i},'.mat');
        save(filename,'Ball_Pos')
        save([surfix,obj{j},'/err2_',robot{i},'.mat'],'err2')
    end
end