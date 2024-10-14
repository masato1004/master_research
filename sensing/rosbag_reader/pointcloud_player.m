%% simulate 32 channel lidar
% figure()
ospc = pointCloud(readXYZ(f_ousMsgs{1}));
data = ospc.Location;
[theta,rho,z] = cart2pol(data(:,1),data(:,2),data(:,3));
phi = atan2(z, rho);
[~,phi_idx] = sort(phi);
data = data(phi_idx,:);
data32 = [];
for i = 0:2:127
    data32 = [data32; data(1024*i+1:1024*(i+1),:)];
end
data32 = pointCloud(data32);
data32 = pctransform(data32,tform_lidar);

%% play
data32 = data32.Location;
data32 = pointCloud(data32(data32(:,2)>=-1.2 & data32(:,2)<=10,:,:));
lidarPlayer = pcplayer(data32.XLimits,data32.YLimits,data32.ZLimits);
view(lidarPlayer, data32);
campos(lidarPlayer.Axes, [0 0 0]);
camtarget(lidarPlayer.Axes, [0 1 0]);
for n = 0:0.05:5
    campos(lidarPlayer.Axes, [0 -n 0]);
    pause(0.1)
end

%% pcplayer
lidarPlayer = pcplayer(ospc.XLimits,ospc.YLimits,ospc.ZLimits);
view(lidarPlayer, ospc);
campos(lidarPlayer.Axes, [0 0 0]);
camtarget(lidarPlayer.Axes, [0 1 0]);
for n = 0:0.05:5
    campos(lidarPlayer.Axes, [0 -n 0]);
    pause(0.1)
end