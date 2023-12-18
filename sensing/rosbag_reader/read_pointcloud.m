%% load tf from calibration file
% transform = readmatrix("/home/inouemasato/ytlab_ros_ws/ytlab_handheld_sensoring_system/ytlab_handheld_sensoring_system_modules/calibration_files/calibration_file_zed.csv");
transform = load("transform.mat","transform").transform;
rotm = quat2rotm(-[transform(7),transform(4:6)]);
% translation = transform(1:3);
translation = [0,0,0];
tform = rigid3d(rotm,translation);

%% ros ouster pointcloud topic
% ros time resolution is 1e-6
% bag = rosbag("E:\nissan\20231209_calibration\_2023-12-09-16-57-02.bag");
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
display("start");
for i = 1:length(oust)
    % display ouster pointcloud
    ospc = pointCloud(readXYZ(ousMsgs{i}));
    % ospc = pctransform(ospc,tform);
    ouspc_show = pcshow(ospc); hold on;

    % display zed2i pointcloud
    zedpc = pointCloud(readXYZ(zedMsgs{i}));
    zedpc = pctransform(zedpc,tform);
    zedpc_show = pcshow(zedpc);
    drawnow

    delete(zedpc_show);
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