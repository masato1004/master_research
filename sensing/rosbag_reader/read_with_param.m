%% rotate with position parameter from memo
rotate_angle_temp = [0 0 0];
translation_temp = [0 -1.755 1.030]; % from memo
tform_temp = rigidtform3d(rotate_angle_temp,translation_temp);
figure
ospc_read = readXYZ(ousMsgs{116}); % 150 -> bar
% ospc = pointCloud(ospc_read(ospc_read(:,2)>=-1.2 & ospc_read(:,2)<=7 & ospc_read(:,1)>=-2 & ospc_read(:,1)<=2,:,:));
ospc = pointCloud(ospc_read(ospc_read(:,2)>=-1.2 & ospc_read(:,2)<=7 & ospc_read(:,1)>=-8 & ospc_read(:,1)<=8,:,:));
ospc = pctransform(ospc,tform_lidar);
%ouspc_show = pcshow(ospc); hold on;
xlabel("\itX \rm[m]");
ylabel("\itY \rm[m]");
zlabel("\itZ \rm[m]");
fontname(gcf,"Times New Roman");
fontsize(gca,16,"points");
% hold on
zedpc = pointCloud(readXYZ(zedMsgs{150}));
zedpc = pctransform(zedpc,tform);
zedpc = pctransform(zedpc,tform_lidar);
zedpc = pctransform(zedpc,tform_temp);
pcshowpair(ospc,zedpc);
%zedpc_show = pcshow(zedpc);
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);