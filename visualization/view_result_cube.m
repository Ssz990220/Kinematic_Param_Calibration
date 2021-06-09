clear;clc;
% Specify folder
% surfix = './experiment/';      % Change this line to match the date and time
surfix = './../../gocator_pcl/src/pcl_pub/results/0429/cube_scan/';
% experiment_number = 6;
batch_size = 39;
mask = [13 14 27 28];
T_path = strcat(surfix,'Raw_Ts_POE.mat');
load(T_path, 'Ts_record');
count = 0;
for number = 1:batch_size
    if any(number == mask)
        continue
    end
    filename = [surfix,num2str(number),'.ply'];
    ptCloud = pcread(filename);
    T = Ts_record{1}(:,:,number);
    tform = rigid3d(T');
    count = count+1;
    ptCloudTrans{count} = pctransform(ptCloud,tform);
end

for i = 2:size(ptCloudTrans,2)
    ptCloudTrans{i} = pcmerge(ptCloudTrans{i-1},ptCloudTrans{i},1);
end

ptCloud = ptCloudTrans{end};
xlimits = ptCloud.XLimits;
ylimits = ptCloud.YLimits;
zlimits = ptCloud.ZLimits;
player = pcplayer(xlimits,ylimits,zlimits);
xlabel(player.Axes,'X (mm)');
ylabel(player.Axes,'Y (mm)');
zlabel(player.Axes,'Z (mm)');

view(player,ptCloud);