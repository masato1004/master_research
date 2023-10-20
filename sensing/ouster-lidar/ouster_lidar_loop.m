exp_purpose = "test";

ousterObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7502);
start(ousterObj);

exp_time = datestr(datetime,'yyyy-mm-dd-HH-MM-ss');
if not(exist("OusterLiDARply/"+exp_purpose+"/"+exp_time,'dir'))
    mkdir("OusterLiDARply/"+exp_purpose+"/"+exp_time)
end
savedir_lidar = "OusterLiDARply/"+exp_purpose+"/"+exp_time+"/";
f = figure('name','Ouster LiDAR : Point Cloud','NumberTitle','off','keypressfcn',@(obj,evt) 0);
scatter(1,1);

key = 1;
% loop over frames, till Esc is pressed
count = 100000000;
tstart = tic;
while (key ~= 27)
    
    [ptCloud,timestamp] = read(ousterObj,"latest");
    % drawnow; %this checks for interrupts
    count = count + 1;

    pcwrite(ptCloud,savedir_lidar+count+"-"+string(datetime('now','TimeZone','local','Format','HH-mm-ss-SSSSS')),PLYformat="binary")

    key = uint8(get(f,'CurrentCharacter'));
    if(~length(key))
        key=0;
    end
end
tend = toc(tstart);
fps=(count-100000000)/tend
close(f);

% pcshow(ptCloud);
% pcviewer(ptCloud);
clear ousterObj;