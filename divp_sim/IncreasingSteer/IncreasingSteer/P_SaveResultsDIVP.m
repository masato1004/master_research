filename = "scenario/test_manhole/scenario_1_divp_Veh_NissanXtrail_1.csv";
opts = detectImportOptions(filename);
opts.SelectedVariableNames = ["timestamp", "pos_x","pos_y","pos_z", "yaw_rad", "pitch_rad", "roll_rad","vel_x", "vel_y", "vel_z", "a_vel_yaw_rad", "a_vel_pitch_rad", "a_vel_roll_rad", "acc_x", "acc_y", "acc_z", "a_acc_yaw_rad", "a_acc_pitch_rad", "a_acc_roll_rad"];
pos_table = readtable(filename,opts);

pos_results = positions';
vel_results = velocitys';
initstate = [pos_table.pos_x(1) pos_table.pos_y(1) pos_table.pos_z(1) pos_table.yaw_rad(1) pos_table.pitch_rad(1) pos_table.roll_rad(1)];
initpos = initstate(1:3);
initagl = [initstate(4) 0 0];
pcdpos = pointCloud([pos_table.pos_x,pos_table.pos_y,pos_table.pos_z]);
R1 = eul2rotm(initagl);
A1 = [[R1;0,0,0],[initpos(1); initpos(2); initpos(3);1]];
pos_results = pctransform(pointCloud([pos_results(:,1)-temp.RearAxlePositionfromCG,-pos_results(:,2),-pos_results(:,3)]),rigidtform3d(A1));
pos_results = pos_results.Location;
vel_results = pctransform(pointCloud([vel_results(:,1),-vel_results(:,2),-vel_results(:,3)]),rigidtform3d(A1));
vel_results = vel_results.Location;

ang_results = angles'; ang_results(:,3) = -ang_results(:,3) + pos_table.yaw_rad(1); ang_results(:,2) = -ang_results(:,2) + pos_table.pitch_rad(1);
ang_results = [ang_results(:,3),ang_results(:,2),ang_results(:,1)];

writematrix(pos_results,'pos.csv')
writematrix(vel_results,'vel.csv')
writematrix(ang_results,'ang.csv')