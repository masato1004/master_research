function [zfl,zfr,zrl,zrr] = F_RoadLoader(FL,FR,RL,RR,pcdmappath,scenariopath)
    
    persistent pcdmap
    if isempty(pcdmap)
        disp("Loading 3D map points...")
        pcdmap = pcread(pcdmappath);
        gridStep = 0.02;
        pcdmap = pcdownsample(pcdmap,'gridAverage',gridStep);
    
        %% if transform is needed
        filename = scenariopath;
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
    
    [k,~] = dsearchn(pcdmap(:,1:2),[FL;FR;RL;RR]);
    zfl = pcdmap(k(1),3);
    zfr = pcdmap(k(2),3);
    zrl = pcdmap(k(3),3);
    zrr = pcdmap(k(4),3);
end