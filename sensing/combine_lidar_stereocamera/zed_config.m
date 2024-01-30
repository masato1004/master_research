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


if(strcmp(result,'SUCCESS'))
    % init point cloud data
    nb_elem = requested_size(1) * requested_size(2);
    pt_X = zeros(requested_size);
    pt_Y = zeros(requested_size);
    pt_Z = zeros(requested_size);
end