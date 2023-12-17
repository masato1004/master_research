%% load tf from calibration file
transform = readmatrix("/home/inouemasato/ytlab_ros_ws/ytlab_handheld_sensoring_system/ytlab_handheld_sensoring_system_modules/calibration_files/calibration_file_zed.csv");
% ros time resolution is 1e-6
bag = rosbag("/home/inouemasato/ytlab_ros_ws/ytlab_rosbag/rosbag/_2023-12-09-16-57-02.bag");
osbag = select(bag,'Topic','/ouster/points');
osMsgs = readMessages(osbag);
osts = timeseries(osbag);
ost = osts.Time;

zedbag = select(bag,'Topic','/zed2i/zed_node/point_cloud/cloud_registered');
zedMsgs = readMessages(zedbag);
zedts = timeseries(zedbag);
zedt = osts.Time;
% pcFilesPath = fullfile(tempdir,'PointClouds');
% 
% if ~exist(pcFilesPath,'dir')
%     mkdir(pcFilesPath);
% end
display("start");
for i = 1:length(ost)
    ospc = pointCloud(readXYZ(osMsgs{i}));
    pcshow(ospc);

    zedpc = pointCloud(readXYZ(zedMsgs{i}));
    pcshow(zedpc);
    drawnow
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