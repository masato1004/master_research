%% Clearing
clc;
warning('off','all');
disp('========= ZED SDK PLUGIN =========');
disp('-- Get 3D Point Cloud --');
close all;
clear mex; clear functions; clear all;

%% mkdir
exp_purpose = "test";
exp_time = datestr(datetime,'yyyy-MM-dd-HH-mm-ss');
if not(exist("OusterLiDARply/"+exp_purpose+"/"+exp_time,'dir'))
    mkdir("OusterLiDARply/"+exp_purpose+"/"+exp_time)
end
if not(exist("OusterIMUply/"+exp_purpose+"/"+exp_time,'dir'))
    mkdir("OusterIMUply/"+exp_purpose+"/"+exp_time)
end
if not(exist("ZED2iply/"+exp_purpose+"/"+exp_time,'dir'))
    mkdir("ZED2iply/"+exp_purpose+"/"+exp_time)
end
savedir_lidar = "OusterLiDARply/"+exp_purpose+"/"+exp_time+"/";
savedir_imu = "OusterIMUply/"+exp_purpose+"/"+exp_time+"/";
savedir_zed = "ZED2iply/"+exp_purpose+"/"+exp_time+"/";

%% configuration of zed camera
% run("zed_config.m");
object_distance = 0-15;  % [m]

cam_angle = 16;    % [deg]
cam_height = 1.45; % [m]

% requested_size = [1700 1200];
requested_size = [750 500];
% requested_size = [640 480];
cResolution = ["2K", "1080HD", "720HD", "VGA"]; cam_resolution = 2; % 1~4
% cResolution = ["2K", "1080HD", "720HD", "VGA"]; cam_resolution = 3; % 1~4


depth_max = 15;  % [m]
depth_min = 0;  % [m]
dMode = ["PERFORMANCE", "QUALITY", "ULTRA", "NEURAL"]; dmode_num = 3;          % 1~4

sMode = ["STANDARD", "FILL"]; smode_num = 2;  % 1~2

%% Camera parameter
InitParameters.camera_resolution = cam_resolution-1; %HD1080
InitParameters.coordinate_units = 2; %METER
InitParameters.depth_mode =  dmode_num; %ULTRA
InitParameters.coordinate_system = 3; %COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP

% DepthClamp value, maximum depth (in METER)
InitParameters.depth_maximum_distance = depth_max;
InitParameters.depth_minimum_distance = depth_min;
result = mexZED('open', InitParameters);
caminfo = mexZED('getCameraInformation'); % checking parameters

%% Run
if not(exist("ZEDply/"+exp_purpose+"/"+exp_time,'dir'))
    mkdir("ZEDply/"+exp_purpose+"/"+exp_time)
end


%% figure for keyboard interrupt
f = figure('name','Ouster LiDAR : Point Cloud','NumberTitle','off','keypressfcn',@(obj,evt) 0);
plot(1,1);
key = 1;

if(strcmp(result,'SUCCESS'))
    % init point cloud data
    nb_elem = requested_size(1) * requested_size(2);
    pt_X = zeros(requested_size);
    pt_Y = zeros(requested_size);
    pt_Z = zeros(requested_size);
    
    % Setup runtime parameters
    RuntimeParameters.sensing_mode = smode_num; % STANDARD sensing mode
end

%% generate objects
lidarObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7502);
% imuObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7503);
start(lidarObj);
% start(imuObj);

%% loop over frames, till Esc is pressed
count = 1000000;
tstart = tic;
while (key ~= 27)
    count = count + 1;

    [lidarCloud,lidartimestamp] = read(lidarObj,"latest");  % get lidar data
    % [imuCloud,imutimestamp] = read(imuObj,"latest");  % get imu data
    result = mexZED('grab', RuntimeParameters);

    if(strcmp(result,'SUCCESS'))  % get stereo camera data
        % get pointcloud data
        [pt_X, pt_Y, pt_Z] = mexZED('retrieveMeasure', 3, requested_size(1), requested_size(2));
        vertices = [reshape(pt_X, [height(pt_X)*width(pt_X),1]),reshape(pt_Y, [height(pt_Y)*width(pt_Y),1]),reshape(pt_Z, [height(pt_Z)*width(pt_Z),1])];
        zedCloud = pointCloud(vertices);
        pcwrite(zedCloud,savedir_zed+count+"-"+datestr(lidartimestamp,"HH-mm-ss_FFF"),PLYformat="binary")

        % get imu data
        sensors_data = mexZED('getSensorsData', 1); % ask CURRENT sensors data
    end

    pcwrite(lidarCloud,savedir_lidar+count+"-"+datestr(lidartimestamp,"HH-mm-ss_FFF"),PLYformat="binary")  % save lidar data as ply
    % pcwrite(imuCloud,"OusterIMUply/"+exp_purpose+"/"+count+"-"+datestr(imutimestamp,"HH-mm-ss_FFF"),PLYformat="binary")  % save lidar data as ply

    % keyboard interrupt
    key = uint8(get(f,'CurrentCharacter'));
    if(~length(key))
        key=0;
    end
end
tend = toc(tstart);
fps=(count-1000000)/tend

%% check the last pointcloud and clear objects
figure(name='ouster LiDAR, IMU and ZED-2i data');
subplot(121); pcshow(lidarCloud);
subplot(122); pcshow(zedCloud);
% subplot(133); pcshow(imuObj);

mexZED('close')
clear lidarObj;
% clear imuObj;
clear mex;
disp('========= END =========');