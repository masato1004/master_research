%% load tf from calibration file
f_tf = readmatrix("calibration_file_zed_nissan_front.csv");
r_tf = readmatrix("calibration_file_zed_nissan_roof.csv");
% transform = load("transform.mat","transform").transform;
f_rotm = quat2rotm([f_tf(7),f_tf(4:6)]);
r_rotm = quat2rotm([r_tf(7),r_tf(4:6)]);
% translation = [transform(2),transform(3)-0.55,transform(1)-0.55];
f_translation = [f_tf(2),f_tf(3),f_tf(1)];
r_translation = [r_tf(2),r_tf(3),r_tf(1)];
% translation = [0,0,0];
f_tform = rigid3d(f_rotm,f_translation);
r_tform = rigid3d(r_rotm,r_translation);

%% lidar inclination
lidar_inc = 20; % [deg]
rotvec = [1,0,0]*deg2rad(lidar_inc);
rotationMatrix = rotvec2mat3d(rotvec);
translation = [0 0 0];
tform_lidar = rigid3d(rotationMatrix,translation);


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
for i = 1:length(oust)
    % display ouster pointcloud
    f_ospc_read = readXYZ(f_ousMsgs{i}); % 130 -> bar
    % ospc = pointCloud(f_ospc_read(f_ospc_read(:,2)>=-1.2 & f_ospc_read(:,2)<=7 & f_ospc_read(:,1)>=-2 & f_ospc_read(:,1)<=2,:,:));
    f_ospc = pointCloud(f_ospc_read(f_ospc_read(:,2)<=1.2 & f_ospc_read(:,2)>=-7,:,:));
    f_ospc = pctransform(f_ospc,f_tform);
    ouspc_show = pcshow(f_ospc); hold on;

    %%%%%%%%%%%%%%%%%%%%%%%% TODO roof lidar points %%%%%%%%%%%%%%%%%%%%%%%%%%%

    % display zed2i pointcloud
    % zedpc = pointCloud(readXYZ(zedMsgs{i}));
    % zedpc = pctransform(zedpc,tform);
    % zedpc = pctransform(zedpc,tform_lidar);
    % zedpc_show = pcshow(zedpc);
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