Exp_purpose = "test";

ousterObj = ousterlidar("OS1-128","ousterlidar_caib.json",Port=7503);
start(ousterObj);

if not(exist("OusterIMUply/"+Exp_purpose,'dir'))
    mkdir("OusterIMUply/"+Exp_purpose)
end
f = figure('name','Ouster IMU : Point Cloud','NumberTitle','off','keypressfcn',@(obj,evt) 0);

key = 1;
% loop over frames, till Esc is pressed
count = 0;
tstart = tic;
while (key ~= 27)
    
    [ptCloud,timestamp] = read(ousterObj,"latest");
    % drawnow; %this checks for interrupts
    count = count + 1;

    pcwrite(ptCloud,"OusterIMUply/"+Exp_purpose+"/"++count+"test",PLYformat="binary")

    key = uint8(get(f,'CurrentCharacter'));
    if(~length(key))
        key=0;
    end
end
tend = toc(tstart);
fps=count/tend

pcshow(ptCloud);
pcviewer(ptCloud);
clear ousterObj;