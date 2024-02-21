%% load tf from calibration file
f_tf = readmatrix("calibration_file_zed_nissan_front.csv");
r_tf = readmatrix("calibration_file_zed_nissan_roof.csv");
f_rotm = quat2rotm([f_tf(7),f_tf(4:6)]);
r_rotm = quat2rotm([r_tf(7),r_tf(4:6)]);
f_translation = [f_tf(1),f_tf(2),f_tf(3)];
r_translation = [r_tf(1),r_tf(2),r_tf(3)];
f_tform = rigidtform3d(f_rotm,f_translation);
r_tform = rigidtform3d(r_rotm,r_translation);

%% lidar inclination
zed_inc_z = 2; % [deg]
zed_inc_y = 14.0375;
rotagl = [0,zed_inc_y,zed_inc_z];
translation = [0 0 0];
tform_lidar = rigidtform3d(rotagl,translation);


bag = rosbag("D:\nissan\20240220_natc_internship\_2024-02-20-15-59-15_drive1_20-15-10.bag");
%% ros ouster-front pointcloud topic
% ros time resolution is 1e-6
f_ousbag = select(bag,'Topic','/ouster_front/points');
f_ousMsgs = readMessages(f_ousbag);
f_ousts = timeseries(f_ousbag);
f_oust = f_ousts.Time;


%% ros ouster-front pointcloud topic
% ros time resolution is 1e-6
r_ousbag = select(bag,'Topic','/ouster_roof/points');
r_ousMsgs = readMessages(r_ousbag);
r_ousts = timeseries(r_ousbag);
r_oust = r_ousts.Time;

%% ros zed2i pointcloud topic
% zedbag = select(bag,'Topic','/zed2i/zed_node/point_cloud/cloud_registered');
% zedMsgs = readMessages(zedbag);
% zedts = timeseries(zedbag);
% zedt = ousts.Time;
% pcFilesPath = fullfile(tempdir,'PointClouds');
% 
% if ~exist(pcFilesPath,'dir')
%     mkdir(pcFilesPath);
% end

%% show
disp("start");
figure;
[min_len,min_idx] = min([length(r_oust),length(f_oust)]);
max_time = max([r_oust(1:end-2),f_oust],[],'all');
min_time = min([r_oust(1:end-2),f_oust],[],'all');

dt = 1e-4;
time_list = min_time:dt:max_time;


round_num = 3;
f_oust_rounded = round(f_oust,round_num);
r_oust_rounded = round(r_oust,round_num);

start_sec = 47.5;
front_pass_data_num = sum(f_oust_rounded < time_list(start_sec/dt));
rear_pass_data_num = sum(r_oust_rounded < time_list(start_sec/dt));
f_oust_rounded = f_oust_rounded(f_oust_rounded > time_list(start_sec/dt));
r_oust_rounded = r_oust_rounded(r_oust_rounded > time_list(start_sec/dt));

f_data_num = front_pass_data_num + 1;
r_data_num = rear_pass_data_num + 1;
f_oust_dnum = 1;
r_oust_dnum = 1;

for i = start_sec/dt:length(time_list)
    if abs(time_list(i) - f_oust_rounded(f_oust_dnum)) < dt/10
            % display ouster-front pointcloud
            f_ospc_read = readXYZ(f_ousMsgs{f_data_num}); % 130 -> bar
            % f_ospc = pointCloud(f_ospc_read(f_ospc_read(:,2)<=1.2 & f_ospc_read(:,2)>=-7 & f_ospc_read(:,1)>=-2 & f_ospc_read(:,1)<=2,:,:));
            % f_ospc = pointCloud(f_ospc_read(f_ospc_read(:,2)<=1.2 & f_ospc_read(:,2)>=-7,:,:));
            f_ospc = pointCloud(f_ospc_read);
            f_ospc = pctransform(f_ospc,f_tform);
            f_ospc = pctransform(f_ospc,tform_lidar);
            f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,1)>=-1.2 & f_ospc.Location(:,1)<=7 & f_ospc.Location(:,2)>=-2 & f_ospc.Location(:,2)<=2,:,:));
            if f_oust_dnum >= 2
                delete(f_ouspc_show);
            end
            f_ouspc_show = pcshow(f_ospc); hold on;
            f_data_num = f_data_num + 1;
            f_oust_dnum = f_oust_dnum + 1;
            view(-90,40)
            drawnow
    end

    if abs(time_list(i) - r_oust_rounded(r_oust_dnum)) < dt/10
            % display ouster-roof pointcloud
            r_ospc_read = readXYZ(r_ousMsgs{r_data_num}); % 130 -> bar
            % r_ospc = pointCloud(r_ospc_read(r_ospc_read(:,1)>=1.5 & r_ospc_read(:,1)<=8.5 & r_ospc_read(:,2)>=-2 & r_ospc_read(:,2)<=2,:,:));
            % r_ospc = pointCloud(r_ospc_read(r_ospc_read(:,1)>=1.5 & r_ospc_read(:,1)<=8.5,:,:));
            r_ospc = pointCloud(r_ospc_read);
            r_ospc = pctransform(r_ospc,r_tform);
            r_ospc = pctransform(r_ospc,tform_lidar);
            r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,1)>=1.5 & r_ospc.Location(:,1)<=8.5 & r_ospc.Location(:,2)>=-2 & r_ospc.Location(:,2)<=2,:,:));
            if r_oust_dnum >= 2
                delete(r_ouspc_show);
            end
            r_ouspc_show = pcshow(r_ospc); hold on;
            r_data_num = r_data_num + 1;
            r_oust_dnum = r_oust_dnum + 1;
            view(-90,40)
            % drawnow
    end
    disp(round(time_list(i)-time_list(1),2)+"[s]--"+ f_data_num + "-" + r_data_num)

    % display zed2i pointcloud
    % zedpc = pointCloud(readXYZ(zedMsgs{i}));
    % zedpc = pctransform(zedpc,tform);
    % zedpc = pctransform(zedpc,tform_lidar);
    % zedpc_show = pcshow(zedpc);

    % delete(zedpc_show);
    % n_strPadded = sprintf('%04d',i) ;
    % pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    % pcwrite(pc,pcFileName);
end

% pclist = dir("C:\Users\masato\AppData\Local\Temp\PointClouds\*.pcd");
% for i = 1:length(pclist)
%     filename = pclist(i).folder + "/" + pclist(i).name;
%     ptcloud = pcread(filename);
%     pcshow(ptcloud);
%     drawnow
% end

%% ego view
% helperVisualizeEgoView(ospc);
%%%
% *|helperVisualizeEgoView|* visualizes point cloud data in the ego
% perspective by rotating about the center.
function player = helperVisualizeEgoView(ptCloud)

% Create a pcplayer object
xlimits = ptCloud.XLimits;
ylimits = ptCloud.YLimits;
zlimits = ptCloud.ZLimits;

player = pcplayer(xlimits, ylimits, zlimits);

% Turn off axes lines
axis(player.Axes, 'off');

% Set up camera to show ego view
camproj(player.Axes, 'perspective');
camva(player.Axes, 90);
campos(player.Axes, [0 0 0]);
camtarget(player.Axes, [-1 0 0]);

% Set up a transformation to rotate by 5 degrees
theta = 5;

eulerAngles = [0 0 theta];
translation = [0 0 0];
rotateByTheta = rigidtform3d(eulerAngles, translation);

for n = 0 : theta : 359
    % Rotate point cloud by theta
    ptCloud = pctransform(ptCloud, rotateByTheta);
    
    % Display point cloud
    view(player, ptCloud);
    
    pause(0.05)
end
end