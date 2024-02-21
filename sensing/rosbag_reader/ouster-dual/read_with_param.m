%% rotate with position parameter from memo
rotate_angle_temp = [0 0 0];
translation_temp = [0 -1.755 1.030]; % from memo
tform_temp = rigidtform3d(rotate_angle_temp,translation_temp);
figure
r_ospc_read = readXYZ(r_ousMsgs{965}); % 965 -> hump
r_ospc = pointCloud(r_ospc_read(r_ospc_read(:,1)>=1.5 & r_ospc_read(:,1)<=8.5 & r_ospc_read(:,2)>=-2 & r_ospc_read(:,2)<=2,:,:));
% r_ospc = pointCloud(r_ospc_read(r_ospc_read(:,1)>=1.5 & r_ospc_read(:,1)<=8.5,:,:));
% r_ospc = pointCloud(r_ospc_read);
r_ospc = pctransform(r_ospc,r_tform);
r_ospc = pctransform(r_ospc,tform_lidar);
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