exp_purpose = "test";

ousterObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7502);
start(ousterObj);

if not(exist("OusterLiDARply/"+exp_purpose,'dir'))
    mkdir("OusterLiDARply/"+exp_purpose)
end
sevedir = "OusterLiDARply/"+exp_purpose+"/";
f = figure('name','Ouster LiDAR : Point Cloud','NumberTitle','off','keypressfcn',@(obj,evt) 0);

key = 1;
% loop over frames, till Esc is pressed
count = 1000000;
tstart = tic;
while (key ~= 27)
    
    [ptCloud,timestamp] = read(ousterObj,"latest");
    % drawnow; %this checks for interrupts
    count = count + 1;

    pcwrite(ptCloud,sevedir+count+"-"+datestr(timestamp,"HH-mm-ss_FFF"),PLYformat="binary")

    key = uint8(get(f,'CurrentCharacter'));
    if(~length(key))
        key=0;
    end
end
tend = toc(tstart);
fps=(count-1000000)/tend

pcshow(ptCloud);
pcviewer(ptCloud);
clear ousterObj;