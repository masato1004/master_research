% ros time resolution is 1e-6
bag = rosbag("E:\nissan\20231209_calibration\_2023-12-09-16-57-02.bag")
osbag = select(bag,'Topic','/ouster/points')
osMsgs = readMessages(osbag);
osts = timeseries(osbag)
ost = osts.Time;

zedbag = select(bag,'Topic','/zed2i/zed_node/point_cloud/cloud_registered')
zedMsgs = readMessages(zedbag);
zedts = timeseries(zedbag)
zedt = osts.Time;
% pcFilesPath = fullfile(tempdir,'PointClouds');
% 
% if ~exist(pcFilesPath,'dir')
%     mkdir(pcFilesPath);
% end
display("start");
for i = 1:length(ost)
    pc = pointCloud(readXYZ(osMsgs{i}));
    pcshow(pc);
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