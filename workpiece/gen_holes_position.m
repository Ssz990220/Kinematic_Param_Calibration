close all
filename ='.\experiment\experiment_0407\workpiece_measurement_1929\workpiece_measurement6_move_up.txt';             % Change this line before you run to read the right file
[p_measure, Ts] = read_real_measure_data(filename);
n_measures = size(Ts, 3)/8;
holes = zeros(3,size(Ts,3));
%% Get hole position
for i = 1:size(Ts,3)
    hole = Ts(:,:,i) * [p_measure(:,i);1];
    hole = hole(1:3);
    holes(:,i) = hole;
end
%% find hole position in each measuring iteration
holes_reorder = zeros(3, n_measures, 8);
for i = 1:8
    for j = 1:n_measures
        holes_reorder(:,j,i) = holes(:,i + (j-1)*8);
    end
end
%% visualize result
index = 1:n_measures;
for i = 1:8
    mean_xyz = mean(holes_reorder(:,:,i),2);
    figure
    hold on
    for j = 1:3
        plot(index, error_xyz(j,:));
    end
    legend('x','y','z');
    ylim([-0.1,0.1]);
end
%% remove error data
mean_xyz_record = zeros(3,8);
measure_mask = 1:n_measures;                    % you can use array to filter one measure of all holes. e.g. [1 2 3 5 6 7] to filter the 4th measurement    
for i = 1:8
    mean_xyz = mean(holes_reorder(:,measure_mask,i),2);
    mean_xyz_record(:,i) = mean_xyz;
    error_xyz = holes_reorder(:,:,i) - mean_xyz;
end
%% remove wrong hole
hole_mask = 1:8;                                          % Same as above, you can use an array to filter one hole in all measurements.
mean_xyz_record = mean_xyz_record(:,hole_mask);
%% Save
save '.\experiment\experiment_0407\workpiece_measurement_1929\workpiece_measurement6_move_up.mat' mean_xyz_record