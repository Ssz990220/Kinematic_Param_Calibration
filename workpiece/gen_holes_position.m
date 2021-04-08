close all
filename ='.\experiment\experiment_0407\workpiece_measurement_1929\workpiece_measurement6_move_up.txt';
[p_measure, Ts] = read_real_measure_data(filename);
n_measures = size(Ts, 3)/8;
holes = zeros(3,size(Ts,3));
for i = 1:size(Ts,3)
    hole = Ts(:,:,i) * [p_measure(:,i);1];
    hole = hole(1:3);
    holes(:,i) = hole;
end

holes_reorder = zeros(3, n_measures, 8);
for i = 1:8
    for j = 1:n_measures
        holes_reorder(:,j,i) = holes(:,i + (j-1)*8);
    end
end

index = 1:n_measures;
mean_xyz_record = zeros(3,8);
for i = 1:8
    mean_xyz = mean(holes_reorder(:,:,i),2);
    mean_xyz_record(:,i) = mean_xyz;
    error_xyz = holes_reorder(:,:,i) - mean_xyz;
    figure
    hold on
    for j = 1:3
        plot(index, error_xyz(j,:));
    end
    legend('x','y','z');
    ylim([-0.1,0.1]);
end
%% Save
% mean_xyz_record = mean_xyz_record(:,[1:3,5:8]);
save '.\experiment\experiment_0407\workpiece_measurement_1929\workpiece_measurement6_move_up.mat' mean_xyz_record