bag = rosbag("E:\nissan\20231209_calibration\_2023-12-09-16-57-02.bag")
pcbag = select(bag,'Topic','/zed2i/zed_node/point_cloud/cloud_registered')
pcMsgs = readMessages(pcbag);
ts = timeseries(pcbag)
t = ts.Time;
pcFilesPath = fullfile(tempdir,'PointClouds');

if ~exist(pcFilesPath,'dir')
    mkdir(pcFilesPath);
end

for i = 1:length(t)
    pc = pointCloud(readXYZ(pcMsgs{i}));
    n_strPadded = sprintf('%04d',i) ;
    pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    pcwrite(pc,pcFileName);
end

pclist = dir("C:\Users\masato\AppData\Local\Temp\PointClouds\*.pcd");
for i = 1:length(pclist)
    filename = pclist(i).folder + "/" + pclist(i).name;
    ptcloud = pcread(filename);
    pcshow(ptcloud);
    drawnow
end