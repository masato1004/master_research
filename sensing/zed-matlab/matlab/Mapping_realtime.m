clc;
disp('========= ZED SDK PLUGIN =========');
disp('-- ZED Camera Spatial Mapping --');
close all;
clear mex; clear functions; clear all;

% initial parameter structure, the same as sl::InitParameters
% values as enum number, defines in : sl/defines.hpp
% or from https://www.stereolabs.com/docs/api/structsl_1_1InitParameters.html

InitParameters.camera_resolution = 1; %HD1080
InitParameters.camera_fps = 60;
InitParameters.coordinate_units = 2; %METER
InitParameters.depth_mode = 1; %PERFORMANCE
InitParameters.coordinate_system = 3; %COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP
%InitParameters.svo_input_filename = '../mySVOfile.svo'; % Enable SVO playback
result = mexZED('open', InitParameters);

if(strcmp(result,'SUCCESS'))
       
    %enable Tracking
    PositionalTrackingParameters.enable_spatial_memory = 1;
    mexZED('enablePositionalTracking', PositionalTrackingParameters);
    
    %enable Spatial Mapping
    SpatialMappingParameters.map_type = 0;
    SpatialMappingParameters.range_meter = 5.;
    SpatialMappingParameters.resolution_meter = 0.08;
    mexZED('enableSpatialMapping', SpatialMappingParameters);
    
    f = figure('name','ZED SDK : Spatial Mapping','NumberTitle','off','keypressfcn',@(obj,evt) 0);
     
    F = [];
    t = datestr(now,'yyyy_mmdd_HHMMSS'); 
    fname = strcat('ZEDmat/realtime',t);
    fname = strcat(fname,'.mat');
%     v = VideoWriter('peaks.avi');
%     open(v);
    gridStep = 0.05;
    thr = 40000;
    key = 1;
    i = 1;
%     f_show(1) = parfeval(backgroundPool, @background_pointcloud, 1, [],F, thr);
    % loop over frames, till Esc is pressed
    tstart = tic;
    while (key ~= 27)       
    % grab the current image and compute the positional tracking
        datas = [];
        result = mexZED('grab');
        if(strcmp(result,'SUCCESS'))
            % retrieve letf image
%             image_left = mexZED('retrieveImage', 0); %left
            %displays it
%             imshow(image_left);
%             if(SpatialMappingParameters.map_type == 0) % Mesh
%             F = fetchOutputs(f_show(i));

            [vertices, ~] = mexZED('extractWholeSpatialMap');
%             cancel(f_show);
%             clear f_show;
%             f_show(i+1) = parfeval(backgroundPool, @background_pointcloud, 1, vertices,F,thr);
            [row_size, ~] = size(vertices);
            
            if row_size > thr
                ptc_ver = pointCloud(vertices(end-thr-1:end, :));
            else
                ptc_ver = pointCloud(vertices);
            end
            F = [F;ptc_ver];
            
%             pcshow(ptc_ver);
%             F = getframe(gcf);
%             writeVideo(v,F)
            % DOWN SAMPLING
%             if row_size ~= 0 
%                 ptCloudA = pcdownsample(ptc_ver,'gridAverage',gridStep);
%                 pcshow(ptCloudA);
%                 view([-1 5 -1.5]);
%                 xlabel('traveling Direction','FontSize',12);
%                 zlabel('Height','FontSize',12);
%                 try
%                     background_ransac(ptCloudA.Location);
%                 catch ME
%                 end
%                 f(ptCloudA.Location) = parfeval(backgroundPool, @background_ransac, 0, ptCloudA.Location);
%                 drawnow;
%             end
%             end
            figure(f);
            drawnow; %this checks for interrupts
%             key = uint8(get(f,'CurrentCharacter'));
            if strcmp(get(f,'currentcharacter'),'q')
%                 close(f);
                save(fname);
                tend = toc(tstart);
                fps=i/tend
                break
            end
            i = i+1
%             if(~length(key))
%                 key=0;
            end
        end
    end
    
    if(SpatialMappingParameters.map_type == 0) % Mesh
        [vertices, faces] = mexZED('extractWholeSpatialMap');
    else % Fused Point Cloud
        [vertices, colors] = mexZED('extractWholeSpatialMap');
    end
    close(f)
% end

% Make sure to call this function to free the memory before use this again
mexZED('close')
disp('========= END =========');
clear mex;