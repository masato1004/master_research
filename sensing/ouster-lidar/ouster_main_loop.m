%% mkdir
exp_purpose = "test";
exp_time = string(datetime);
if not(exist("OusterLiDARply/"+exp_purpose+"/"+exp_time,'dir'))
    mkdir("OusterLiDARply/"+exp_purpose+"/"+exp_time)
end
if not(exist("OusterIMUply/"+exp_purpose+"/"+exp_time,'dir'))
    mkdir("OusterIMUply/"+exp_purpose+"/"+exp_time)
end
savedir_lidar = "OusterLiDARply/"+exp_purpose+"/"+exp_time+"/";
savedir_imu = "OusterIMUply/"+exp_purpose+"/"+exp_time+"/";

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
    
    [lidarCloud,lidartimestamp] = read(lidarObj,"latest");  % get lidar data
    [imuCloud,imutimestamp] = read(imuObj,"latest");  % get imu data
    
    count = count + 1;

    pcwrite(lidarCloud,savedir_lidar+count+"-"+datestr(lidartimestamp,"HH-mm-ss_FFF"),PLYformat="binary")  % save lidar data as ply
    pcwrite(imuCloud,savedir_imu+count+"-"+datestr(imutimestamp,"HH-mm-ss_FFF"),PLYformat="binary")  % save lidar data as ply

    % keyboard interrupt
    key = uint8(get(f,'CurrentCharacter'));
    if(~length(key))
        key=0;
    end
end
tend = toc(tstart);
fps=(count-1000000)/tend

%% check the last pointcloud and clear objects
close f;
figure(name='ouster LiDAR and IMU');
subplot(121); pcshow(lidarCloud);
subplot(122); pcshow(imuObj);
clear lidarObj;
clear imuObj;