% clear all;
warning("off",'all');
addpath("./module_V-Drive");
addpath("./module_Optimization");

% global pcdmap
% if isempty(pcdmap)
%     disp("Loading 3D map points...")
%     pcdmap = pcread("map_gt.ply");
%     gridStep = 0.02;
%     pcdmap = pcdownsample(pcdmap,'gridAverage',gridStep);
% 
%     %% if transform is needed
%     filename = "scenario_1_divp_Veh_NissanXtrail_1.csv";
%     opts = detectImportOptions(filename);
%     opts.SelectedVariableNames = ["timestamp", "pos_x","pos_y","pos_z", "yaw_rad", "pitch_rad", "roll_rad"];
%     pos_table = readtable(filename,opts);
% 
%     initstate = [pos_table.pos_x(1) pos_table.pos_y(1) pos_table.pos_z(1) pos_table.yaw_rad(1) pos_table.pitch_rad(1) pos_table.roll_rad(1)];
%     initpos = initstate(1:3);
%     initagl = [initstate(4) 0 0];
% 
%     R1 = eul2rotm(-initagl);
%     A1 = [[R1;0,0,0],[0; 0; 0;1]];
% 
%     pcdmap = pctransform(pointCloud([pcdmap.Location(:,1)-initpos(1),pcdmap.Location(:,2)-initpos(2),pcdmap.Location(:,3)-initpos(3)]),rigidtform3d(A1));
% 
%     pcdmap = pcdmap.Location;
%     disp("Finish Loading 3D map points.")
% end

%% Load python function
python_path = "C:\Users\"+getenv('username')+"\research\divpenv\Scripts\python.exe";
if pyenv().Executable ~= python_path
    pe = pyenv(Version=python_path);
end
pymod = py.importlib.import_module('F_depthcompletion');
py.importlib.reload(pymod);

%% camera parameter
cam_params = struct();
cam_params.camera_position = [1.881159, 0.0, 1.554000];
cam_params.img_w = 3840;
cam_params.img_h = 2160;
cam_params.k1 = 1.36648;
cam_params.k2 = 1.79417;
cam_params.k3 = 0.1704;
cam_params.k4 = 1.90693;
cam_params.k5 = 2.64875;
cam_params.k6 = 0.97058;
cam_params.p1 = 0.00014;
cam_params.p2 = -0.00008;
cam_params.fx = 2445.66438;
cam_params.fy = 2444.75377;
cam_params.cx = 1905.44853;
cam_params.cy = 1073.60153;
cam_params.imageSize = [cam_params.img_h, cam_params.img_w];
cam_params.focalLength      = [cam_params.fx, cam_params.fy];
cam_params.principalPoint   = [cam_params.cx, cam_params.cy];
cam_params.RadialDistortion6 = [cam_params.k1, cam_params.k2, cam_params.k3, cam_params.k4, cam_params.k5, cam_params.k6];
cam_params.TangentialDistortion = [cam_params.p1,cam_params.p2];

%% depth completion parameters
dc_params = struct();
dc_params.original_img_w = 3840;
dc_params.original_img_h = 2160;
dc_params.crop_h = 552;
dc_params.crop_w = 1496;
dc_params.start_x = 1172+1;
dc_params.start_y = 1271+1;
dc_params.rect_width = 1496-1;
dc_params.rect_height = 552-1;
dc_params.maxCameraDepth = 20;
dc_params.crop_info = {start_y:start_y+rect_height;start_x:start_x+rect_width};

%% optimization parameters
opt_params = struct();
opt_params.tw = 1.485; % Track width
opt_params.wb = 2.86; % Wheelbase
opt_params.v = 50/3.6; % Constant velocity
opt_params.L = 3; % Wheelbase
opt_params.lane_width = 3.5; % Width of the lane
opt_params.g = 9.8;
opt_params.N = 9; % Number of steps
opt_params.interp_steps = 2;
opt_params.dt = 0.175;
opt_params.plane_threshold = 0.002;

%% find initial position
filename = "scenario/test_manhole/scenario_1_divp_Veh_NissanXtrail_1.csv";
opts = detectImportOptions(filename);
opts.SelectedVariableNames = ["timestamp", "pos_x","pos_y","pos_z", "yaw_rad", "pitch_rad", "roll_rad"];
pos_table = readtable(filename,opts);
initstate = [pos_table.pos_x(1) pos_table.pos_y(1) pos_table.pos_z(1) pos_table.yaw_rad(1) pos_table.pitch_rad(1) pos_table.roll_rad(1)];
initpos = initstate(1:3);
initagl = [initstate(4) 0 0];
pcdpos = pointCloud([pos_table.pos_x,pos_table.pos_y,pos_table.pos_z]);
R1 = eul2rotm(-initagl);
A1 = [[R1;0,0,0],[0; 0; 0;1]];
pcdpos = pctransform(pointCloud([pcdpos.Location(:,1)-initpos(1),pcdpos.Location(:,2)-initpos(2),pcdpos.Location(:,3)-initpos(3)]),rigidtform3d(A1));
pcdpos = pcdpos.Location;
[k,~] = dsearchn(pcdpos(:,1),590);
initialpos = pcdpos(k(1),:);

mdl = "System/ISReferenceApplication";
open_system(mdl)
simIn = Simulink.SimulationInput(mdl);
simIn = setModelParameter(simIn,"Solver","ode8","StopTime","20",'FixedStep','1e-3');

mdlwks = get_param('ISReferenceApplication','ModelWorkspace');
temp = getVariable(mdlwks,'VEH');
% Vehicle parameters
temp.InitialLongPosition = initialpos(1);
temp.InitialLatPosition = -initialpos(2);
temp.InitialVertPosition = -initialpos(3)-temp.HeightCG;
temp.InitialRollAngle = pos_table.roll_rad(k);
temp.InitialPitchAngle = -pos_table.pitch_rad(k);
temp.InitialYawAngle = -pos_table.yaw_rad(k)+pos_table.yaw_rad(1);
temp.InitialLongVel = 0;
temp.initpos = initpos;
temp.initagl = initagl;
temp.initstate = initstate;
temp.WheelBase = 2.860;
temp.FrontAxlePositionfromCG = 1.43;
temp.RearAxlePositionfromCG = 1.43;
temp.TrackWidth = 1.485;
temp.VehicleWidth = 1.731;
temp.VehicleLength = 4.769;
temp.diameter = 0.653;
temp.DrawFreq = 0.1;
temp.T_ref = pcdpos;
% temp.T_ref = new_path;
% temp.pcdmap = double(pcdmap);

% Simulation parameters
temp.cam_params = cam_params;
temp.dc_params = dc_params;
temp.opt_params = opt_params;

assignin(mdlwks,'temp',temp);
% global video
out = sim(simIn);
run("P_WheelDisp.m")
% [VEH.InitialLongPosition,VEH.InitialLatPosition,VEH.InitialVertPosition]
% [temp.InitialLongPosition,temp.InitialLatPosition,temp.InitialVertPosition]