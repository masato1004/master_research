%% Clearing
clc;
disp('========= ZED SDK PLUGIN =========');
disp('-- Get 3D Point Cloud --');
close all;
clear mex; clear functions; clear all;

%% Experiment parameter
Exp_purpose = "-20230227tile-";

object_distance = 0-15;  % [m]

cam_angle = 16;    % [deg]
cam_height = 1.45; % [m]

% requested_size = [1700 1200];
requested_size = [750 500];
% requested_size = [640 480];
cResolution = ["2K", "1080HD", "720HD", "VGA"]; cam_resolution = 1; % 1~4
% cResolution = ["2K", "1080HD", "720HD", "VGA"]; cam_resolution = 3; % 1~4

depth_max = 15;  % [m]
depth_min = 0;  % [m]
dMode = ["PERFORMANCE", "QUALITY", "ULTRA", "NEURAL"]; dmode_num = 3;          % 1~4

sMode = ["STANDARD", "FILL"]; smode_num = 2;  % 1~2

% initial parameter structure, the same as sl::InitParameters
% values as enum number, defines in : sl/defines.hpp
% or from https://www.stereolabs.com/docs/api/structsl_1_1InitParameters.html

%% Camera parameter
InitParameters.camera_resolution = cam_resolution-1; %HD1080
InitParameters.coordinate_units = 2; %METER
InitParameters.depth_mode =  dmode_num; %ULTRA
InitParameters.coordinate_system = 3; %COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP
%InitParameters.svo_input_filename = '../mySVOfile.svo'; % Enable SVO playback

% DepthClamp value, maximum depth (in METER)
InitParameters.depth_maximum_distance = depth_max;
InitParameters.depth_minimum_distance = depth_min;
result = mexZED('open', InitParameters);
caminfo = mexZED('getCameraInformation'); % checking parameters

%% Run
if not(exist("ZEDply/"+Exp_purpose,'dir'))
    mkdir("ZEDply/"+Exp_purpose)
end
if(strcmp(result,'SUCCESS'))
    % Create Figure
    f = figure('name','ZED SDK : Point Cloud','NumberTitle','off','keypressfcn',@(obj,evt) 0);
    %create 2 sub figure
%     ha1 = axes('Position',[0.05,0.7,0.9,0.25]);
%     ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
%     axis([-depth_max, depth_max, 0, depth_max, -depth_max ,depth_max])
%     xlabel('X');
%     ylabel('Z');
%     zlabel('Y');
    grid on;
    hold on;
    
    % init point cloud data
    nb_elem = requested_size(1) * requested_size(2);
    pt_X = zeros(requested_size);
    pt_Y = zeros(requested_size);
    pt_Z = zeros(requested_size);
    
    % Setup runtime parameters
    RuntimeParameters.sensing_mode = smode_num; % STANDARD sensing mode

    t = datestr(now,'yyyy_mmdd_HHMMSS'); 
%     fname = strcat("ZEDmat/depthcloud-"+object_distance+"m-",t);
%     fname = strcat(fname,'.mat');
    
    h = plot3(reshape(pt_X, 1,nb_elem), reshape( pt_Y, 1,nb_elem), reshape( pt_Z, 1,nb_elem), '.');
    
    key = 1;
    % loop over frames, till Esc is pressed
    count = 0;
    tstart = tic;
    while (key ~= 27)
        % grab the current image and compute the depth
        result = mexZED('grab', RuntimeParameters);
        if(strcmp(result,'SUCCESS'))
            % retrieve letf image
%             image_left = mexZED('retrieveImage', 0); %left
            %displays it
%             axes(ha1);
%             imshow(image_left);

            % retrieve the point cloud, resized
            [pt_X, pt_Y, pt_Z] = mexZED('retrieveMeasure', 3, requested_size(1), requested_size(2)); %XYZ pointcloud

            %displays it
%             axes(ha2);
%             set(h,'XData',reshape(pt_X, 1,nb_elem))
%             set(h,'YData',reshape(pt_Y, 1,nb_elem))
%             set(h,'ZData',reshape(pt_Z, 1,nb_elem))  
%             subplot(1,1,1)
%             axis on;
%             pt = [reshape(pt_X, [height(pt_X)*width(pt_X),1]),reshape(pt_Y, [height(pt_Y)*width(pt_Y),1]),reshape(pt_Z, [height(pt_Z)*width(pt_Z),1])];
%             pcshow(pointCloud(pt))

            drawnow; %this checks for interrupts
            count = count + 1;

            vertices = [reshape(pt_X, [height(pt_X)*width(pt_X),1]),reshape(pt_Y, [height(pt_Y)*width(pt_Y),1]),reshape(pt_Z, [height(pt_Z)*width(pt_Z),1])];
            pt_output = pointCloud(vertices);
            pcwrite(pt_output,"ZEDply/"+count+"test",PLYformat="binary")

            key = uint8(get(f,'CurrentCharacter'));
            if(~length(key))
                key=0;
            end
        end
    end
    tend = toc(tstart);
    fps=count/tend
    close(f)
end

%% Close mexZED
% Make sure to call this function to free the memory before use this again
mexZED('close')
disp('========= END =========');
clear mex;

%% Drawing
vertices = [reshape(pt_X, [height(pt_X)*width(pt_X),1]),reshape(pt_Y, [height(pt_Y)*width(pt_Y),1]),reshape(pt_Z, [height(pt_Z)*width(pt_Z),1])];
pt_output = pcshow(pointCloud(vertices));

%% save
% if not(exist("ZEDimage/t"+Exp_purpose,'dir'))
%     mkdir("ZEDimage/t"+Exp_purpose)
% end
if not(exist("ZEDfigure/"+Exp_purpose,'dir'))
    mkdir("ZEDimage/"+Exp_purpose)
end

figdir_name = "ZEDfigure/"+Exp_purpose+"/camH-"+cam_height+"-camA-"+cam_angle+"-objD-"+object_distance;
% if not(exist("ZEDimage/t"+Exp_purpose+"/camH-"+cam_height+"-camA-"+cam_angle+"-objD-"+object_distance,'dir'))
%     mkdir("ZEDimage/t"+Exp_purpose+"/camH-"+cam_height+"-camA-"+cam_angle+"-objD-"+object_distance)
% end
if not(exist(figdir_name,'dir'))
    mkdir(figdir_name)
end
figdir_name2 = figdir_name+"/"+"min-"+depth_min+"-max-"+depth_max+"-dMode-"+dMode(dmode_num)+"sMode"+sMode(smode_num)+"-ptcSize-"+requested_size(1)+"_"+requested_size(2)+"-camReso-"+cResolution(cam_resolution);
if not(exist(figdir_name2,'dir'))
    mkdir(figdir_name2)
end

fig_name = figdir_name2+"/"+t;
saveas(pt_output,fig_name+"pt.fig");
save(fig_name+"mt.mat");