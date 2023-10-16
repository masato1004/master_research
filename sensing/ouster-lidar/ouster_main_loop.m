%% mkdir
exp_purpose = "test";
exp_time = string(datetime);
if not(exist("OusterLiDARply/"+exp_purpose+"/"+exp_time,'dir'))
    mkdir("OusterLiDARply/"+exp_purpose+"/"+exp_time)
end
if not(exist("OusterIMUply/"+exp_purpose,'dir'))
    mkdir("OusterIMUply/"+exp_purpose)
end


%% generate objects
lidarObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7502);
imuObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7503);
start(lidarObj);
start(imuObj);

%% figure for keyboard interrupt
f = figure('name','Ouster LiDAR : Point Cloud','NumberTitle','off','keypressfcn',@(obj,evt) 0);
key = 1;

%% loop over frames, till Esc is pressed
count = 0;
tstart = tic;
while (key ~= 27)
    
    [lidarCloud,lidartimestamp] = read(lidarObj,"latest");  % get lidar data
    [imuCloud,imutimestamp] = read(imuObj,"latest");  % get imu data
    
    count = count + 1;

    pcwrite(lidarCloud,"OusterLiDARply/"+exp_purpose+"/"+count+"test",PLYformat="binary")  % save lidar data as ply
    pcwrite(imuCloud,"OusterIMUply/"+exp_purpose+"/"+count+"test",PLYformat="binary")  % save lidar data as ply

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
figure(name='ouster LiDAR and IMU');
subplot(121); pcshow(lidarCloud);
subplot(122); pcshow(imuObj);
clear lidarObj;
clear imuObj;