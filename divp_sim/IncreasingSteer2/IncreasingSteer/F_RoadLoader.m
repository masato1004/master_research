function [zfl,zfr,zrl,zrr] = F_RoadLoader(FL,FR,RL,RR,pcdmappath,scenariopath)
    
    persistent pcdmap
    if isempty(pcdmap)
        disp("Loading 3D map points...")
        pcdmap = pcread(pcdmappath);
        gridStep = 0.01;
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
    
    % [k,~] = dsearchn(pcdmap(:,1:2),[FL;FR;RL;RR]);
    persistent zfl_last zfr_last zrl_last zrr_last
    if sum(FL) ~= 0
        zfl = mean(pcdmap(pcdmap(:,1)<FL(1)+0.025&pcdmap(:,1)>FL(1)-0.025&pcdmap(:,2)<FL(2)+0.025&pcdmap(:,2)>FL(2)-0.025,3));
        zfr = mean(pcdmap(pcdmap(:,1)<FR(1)+0.025&pcdmap(:,1)>FR(1)-0.025&pcdmap(:,2)<FR(2)+0.025&pcdmap(:,2)>FR(2)-0.025,3));
        zrl = mean(pcdmap(pcdmap(:,1)<RL(1)+0.025&pcdmap(:,1)>RL(1)-0.025&pcdmap(:,2)<RL(2)+0.025&pcdmap(:,2)>RL(2)-0.025,3));
        zrr = mean(pcdmap(pcdmap(:,1)<RR(1)+0.025&pcdmap(:,1)>RR(1)-0.025&pcdmap(:,2)<RR(2)+0.025&pcdmap(:,2)>RR(2)-0.025,3));
    end
    if sum(FL) == 0 || isnan(zfl)
        zfl = zfl_last;
    end
    if sum(FR) == 0 || isnan(zfr)
        zfr = zfr_last;
    end
    if sum(RL) == 0 || isnan(zrl)
        zrl = zrl_last;
    end
    if sum(RR) == 0 || isnan(zrr)
        zrr = zrr_last;
    end
    zfl_last = zfl;
    zfr_last = zfr;
    zrl_last = zrl;
    zrr_last = zrr;
end