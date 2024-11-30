% clear all;
warning("off",'all');

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

%% find initial position
filename = "scenario_1_divp_Veh_NissanXtrail_1.csv";
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
[k,~] = dsearchn(pcdpos(:,1),30);
initialpos = pcdpos(k(1),:);

mdl = "System/ISReferenceApplication";
open_system(mdl)
simIn = Simulink.SimulationInput(mdl);
simIn = setModelParameter(simIn,"Solver","ode4","StopTime","20",'FixedStep','1e-3');

mdlwks = get_param('ISReferenceApplication','ModelWorkspace');
temp = getVariable(mdlwks,'VEH');
temp.InitialLongPosition = initialpos(1);
temp.InitialLatPosition = -initialpos(2);
temp.InitialVertPosition = initialpos(3);
temp.InitialRollAngle = pos_table.roll_rad(1);
temp.InitialPitchAngle = pos_table.pitch_rad(1);
temp.InitialYawAngle = 0;
temp.InitialLongVel = 0;
temp.WheelBase = 2.860;
temp.FrontAxlePositionfromCG = 1.43;
temp.RearAxlePositionfromCG = 1.43;
temp.TrackWidth = 1.485;
temp.VehicleWidth = 1.731;
temp.VehicleLength = 4.769;
temp.diameter = 0.653;
temp.DrawFreq = 0.1;
temp.T_ref = pcdpos;
% temp.pcdmap = double(pcdmap);
assignin(mdlwks,'temp',temp);
global video
out = sim(simIn);
% [VEH.InitialLongPosition,VEH.InitialLatPosition,VEH.InitialVertPosition]
% [temp.InitialLongPosition,temp.InitialLatPosition,temp.InitialVertPosition]