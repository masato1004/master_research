%% load tf from calibration file
% transform = readmatrix("/home/inouemasato/ytlab_ros_ws/ytlab_handheld_sensoring_system/ytlab_handheld_sensoring_system_modules/calibration_files/calibration_file_zed.csv");
transform = load("transform.mat","transform").transform;
rotm = quat2rotm([transform(7),transform(4:6)]);
% translation = [transform(2),transform(3)-0.55,transform(1)-0.55];
% translation = [transform(2),transform(3),transform(1)];
translation = [0,0,0];
tform = rigid3d(rotm,translation);

%% lidar inclination
lidar_inc = 20; % [deg]
rotvec = [1,0,0]*deg2rad(lidar_inc);
rotationMatrix = rotvec2mat3d(rotvec);
translation = [0 0 0];
tform_lidar = rigid3d(rotationMatrix,translation);

%% ros ouster pointcloud topic
% ros time resolution is 1e-6
bag = rosbag("E:\nissan\20231221_oppama_trial\_2023-12-21-16-25-08bar_high.bag");
ousbag = select(bag,'Topic','/ouster/points');
ousMsgs = readMessages(ousbag);
ousts = timeseries(ousbag);
oust = ousts.Time;

%% ros zed2i pointcloud topic
zedbag = select(bag,'Topic','/zed2i/zed_node/point_cloud/cloud_registered');
zedMsgs = readMessages(zedbag);
zedts = timeseries(zedbag);
zedt = ousts.Time;
% pcFilesPath = fullfile(tempdir,'PointClouds');
% 
% if ~exist(pcFilesPath,'dir')
%     mkdir(pcFilesPath);
% end

%% show
disp("start");
figure;
for i = 1:length(oust)
    % display ouster pointcloud
    ospc_read = readXYZ(ousMsgs{i}); % 130 -> bar
    % ospc = pointCloud(ospc_read(ospc_read(:,2)>=-1.2 & ospc_read(:,2)<=7 & ospc_read(:,1)>=-2 & ospc_read(:,1)<=2,:,:));
    ospc = pointCloud(ospc_read(ospc_read(:,2)>=-1.2 & ospc_read(:,2)<=7,:,:));
    ospc = pctransform(ospc,tform_lidar);
    ouspc_show = pcshow(ospc); hold on;

    % display zed2i pointcloud
    zedpc = pointCloud(readXYZ(zedMsgs{i}));
    zedpc = pctransform(zedpc,tform);
    zedpc = pctransform(zedpc,tform_lidar);
    zedpc_show = pcshow(zedpc);
    drawnow

    % delete(zedpc_show);
    delete(ouspc_show);
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
helperVisualizeEgoView(ospc);
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