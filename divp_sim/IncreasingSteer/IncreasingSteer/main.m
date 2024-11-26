% clear all;
global pcdmap
if isempty(pcdmap)
    disp("Loading 3D map points...")
    pcdmap = pcread("map_gt.ply");
    gridStep = 0.02;
    pcdmap = pcdownsample(pcdmap,'gridAverage',gridStep);

    %% if transform is needed
    filename = "scenario_1_divp_Veh_NissanXtrail_1.csv";
    opts = detectImportOptions(filename);
    opts.SelectedVariableNames = ["timestamp", "pos_x","pos_y","pos_z", "yaw_rad", "pitch_rad", "roll_rad"];
    pos_table = readtable(filename,opts);
    
    initstate = [pos_table.pos_x(1) pos_table.pos_y(1) pos_table.pos_z(1) pos_table.yaw_rad(1) pos_table.pitch_rad(1) pos_table.roll_rad(1)];
    initpos = initstate(1:3);
    initagl = [initstate(4) 0 0];

    R1 = eul2rotm(-initagl);
    A1 = [[R1;0,0,0],[0; 0; 0;1]];

    pcdmap = pctransform(pointCloud([pcdmap.Location(:,1)-initpos(1),pcdmap.Location(:,2)-initpos(2),pcdmap.Location(:,3)-initpos(3)]),rigidtform3d(A1));

    pcdmap = pcdmap.Location;
    disp("Finish Loading 3D map points.")
end

mdl = "System/ISReferenceApplication";
open_system(mdl)
simIn = Simulink.SimulationInput(mdl);
simIn = setModelParameter(simIn,"Solver","ode8","StopTime","10");

mdlwks = get_param('ISReferenceApplication','ModelWorkspace');
temp = getVariable(mdlwks,'VEH');
temp.InitialLatPosition = 0;
temp.InitialLongVel = 0;
% temp.pcdmap = double(pcdmap);
assignin(mdlwks,'temp',temp);

out = sim(simIn);
% [VEH.InitialLongPosition,VEH.InitialLatPosition,VEH.InitialVertPosition]
% [temp.InitialLongPosition,temp.InitialLatPosition,temp.InitialVertPosition]