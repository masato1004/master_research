%% Clearing
clc;
disp('========= ZED SDK PLUGIN =========');
disp('-- Get 3D Point Cloud --');
close all;
clear mex; clear functions; clear all;

%% mkdir
exp_purpose = "test";
exp_time = string(datetime);
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
run("zed_config.m");

%% generate objects
lidarObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7502);
imuObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7503);
start(lidarObj);
start(imuObj);

%% figure for keyboard interrupt
f = figure('name','Ouster LiDAR : Point Cloud','NumberTitle','off','keypressfcn',@(obj,evt) 0);
key = 1;

%% loop over frames, till Esc is pressed
count = 1000000;
tstart = tic;
while (key ~= 27)
    count = count + 1;

    [lidarCloud,lidartimestamp] = read(lidarObj,"latest");  % get lidar data
    [imuCloud,imutimestamp] = read(imuObj,"latest");  % get imu data
    result = mexZED('grab', RuntimeParameters);

    if(strcmp(result,'SUCCESS'))  % get stereo camera data
        % get pointcloud data
        [pt_X, pt_Y, pt_Z] = mexZED('retrieveMeasure', 3, requested_size(1), requested_size(2));
        vertices = [reshape(pt_X, [height(pt_X)*width(pt_X),1]),reshape(pt_Y, [height(pt_Y)*width(pt_Y),1]),reshape(pt_Z, [height(pt_Z)*width(pt_Z),1])];
        zedCloud = pointCloud(vertices);
        pcwrite(zedCloud,savedir_zed+count+"-"+imutimestamp,PLYformat="binary")

        % get imu data
        sensors_data = mexZED('getSensorsData', 1); % ask CURRENT sensors data
    end

    pcwrite(lidarCloud,"OusterLiDARply/"+exp_purpose+"/"+count+"-"+lidartimestamp,PLYformat="binary")  % save lidar data as ply
    pcwrite(imuCloud,"OusterIMUply/"+exp_purpose+"/"+count+"-"+imutimestamp,PLYformat="binary")  % save lidar data as ply

    % keyboard interrupt
    key = uint8(get(f,'CurrentCharacter'));
    if(~length(key))
        key=0;
    end
end
tend = toc(tstart);
fps=count/tend

%% check the last pointcloud and clear objects
close f;
figure(name='ouster LiDAR, IMU and ZED-2i data');
subplot(131); pcshow(lidarCloud);
subplot(132); pcshow(zedCloud);
subplot(133); pcshow(imuObj);

mexZED('close')
clear lidarObj;
clear imuObj;
clear mex;
disp('========= END =========');