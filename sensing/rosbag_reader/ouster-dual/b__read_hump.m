close all;
warning('off','all');
%% set infomations
msg_nums = [1274 1275];
gridStep = 0.05;

% install params
f_ousmsg_num = msg_nums(1);
r_ousmsg_num = msg_nums(2);

% movmean setting
mean_data_num = [30, 30];

%% correct road surface profile
max_z0 = 0.025;                                                                % [m] max road displacement
ld = [0.05 0.15 0.05];
start_disturbance = 2.5;                                                                  % amplitude
max_distance = 30;                                                           % [m] driving mileage
f_dis_total = [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
r_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
road_total = [0,0,max_z0,max_z0,0,0];  % converting front disturbance and buffer ([m])

%% front point Cloud
f_ospc_read = readXYZ(f_ousMsgs{f_ousmsg_num}); % 965 -> hump
f_ospc = pointCloud(f_ospc_read);

% simulate 32 channel lidar
[~,rho,z] = cart2pol(f_ospc_read(:,1),f_ospc_read(:,2),f_ospc_read(:,3));
phi = atan2(z, rho);
[phi,phi_idx] = sort(phi);
min_rad = deg2rad(-22.5);
max_rad = deg2rad(22.5);
diff_rad = deg2rad(45/128);
data = f_ospc_read(phi_idx,:);
data32 = [];
for i = 0:2:127
    data32 = [data32; data(phi >= min_rad+i*diff_rad & phi < min_rad+(i+1)*diff_rad,:)];
end

f_ospc = pointCloud(data32);

f_ospc = pctransform(f_ospc,f_tform);
f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,1)>=1.2 & f_ospc.Location(:,2)>=-4 & f_ospc.Location(:,2)<=4 & f_ospc.Location(:,3)<=5,:,:));

% depth = 8;
% mesh2 = pc2surfacemesh(pointCloud([f_ospc.Location(:,1),f_ospc.Location(:,2),f_ospc.Location(:,3)]),"poisson",depth);
% datas = mesh2.Vertices;
% xlist = 1.5:0.05:8;
% ylist = -4:0.05:4;
% [xq,yq] = meshgrid(xlist,ylist);
% elevation_mesh = griddata(double(datas(:,1)),double(datas(:,2)),double(datas(:,3)),xq,yq,"natural");
% grid_data = reshape([xq,yq,elevation_mesh],[height(elevation_mesh)*width(elevation_mesh),3]);

f_downptCloud = pcdownsample(f_ospc,'gridAverage',gridStep);
[f_ospc, plane, plane_tform] = fitplane(f_ospc,f_downptCloud,0.01);
f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,3)>=-1 & f_ospc.Location(:,3)<=0.2 & f_ospc.Location(:,2)>=-2 & f_ospc.Location(:,2)<=2 & f_ospc.Location(:,1)<=8.5,:,:));

figure
f_ouspc_show = pcshow(f_ospc);
xlabel("\itX \rm[m]");
ylabel("\itY \rm[m]");
zlabel("\itZ \rm[m]");
fontname(gcf,"Times New Roman");
fontsize(gca,16,"points");
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

%% rear point Cloud
r_ospc_read = readXYZ(r_ousMsgs{r_ousmsg_num}); % 965 -> hump
r_ospc = pointCloud(r_ospc_read);
r_ospc = pctransform(r_ospc,r_tform);
r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,1)>=1 & r_ospc.Location(:,2)>=-4 & r_ospc.Location(:,2)<=4,:,:));
r_downptCloud = pcdownsample(r_ospc,'gridAverage',gridStep);
[r_ospc, ~, plane_tform] = fitplane(r_ospc,r_downptCloud,0.01);
r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,3)>=-0.07 & r_ospc.Location(:,3)<=0.2 & r_ospc.Location(:,2)>=-2 & r_ospc.Location(:,2)<=2 & r_ospc.Location(:,1)<=8.5,:,:));
figure
r_ouspc_show = pcshow(r_ospc);
xlabel("\itX \rm[m]");
ylabel("\itY \rm[m]");
zlabel("\itZ \rm[m]");
fontname(gcf,"Times New Roman");
fontsize(gca,16,"points");
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

%% PICK UP AS 2D
figure();

range_min = 0;        % minimum measurable distance [m]
range_max = 8;        % maximum measurable distance [m]
pick_up_width = 1;  % width of datas for a road profile [m]
pick_up_center = 0;   % center of pick up position [m]

p_min = pick_up_center - pick_up_width/2;
p_max = pick_up_center + pick_up_width/2;

% front
f_line = f_ospc.Location(f_ospc.Location(:,2)>=p_min & f_ospc.Location(:,2)<=p_max & f_ospc.Location(:,1)<=range_max & f_ospc.Location(:,1)>=range_min,:,:);
% f_line = f_ospc.Location(f_ospc.Location(:,1)>=-0.075 & f_ospc.Location(:,1)<=0.075 & f_ospc.Location(:,2)<=7 & f_ospc.Location(:,2)>=5.06,:,:);
[~,f_ind] = sort(f_line(:,1));
f_prev_profile=f_line(f_ind,[true false true])';

f_dis_total_p = [f_dis_total, f_prev_profile(1,:)];
[f_dis_total_p,f_dis_idx] = sort(f_dis_total_p);
f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);

f_ax = subplot(211);% front
f_sc = scatter(f_prev_profile(1,:),f_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#0000ff","DisplayName","Raw Data"); hold on;  % picked up points
f_correct_road = plot(f_dis_total_p,f_correct_road_p,"LineWidth",2,"Color","#aaaaaa","DisplayName","Actual Road"); hold on;
f_pl = plot(f_prev_profile(1,:),movmean(f_prev_profile(2,:),mean_data_num),"LineWidth",2,"Color","#ff0000","DisplayName","Moving Average"); % moving average
grid on;
xlim(f_ax,[range_min,range_max]);
ylim(f_ax,[-0.08, 0.08]);
% axis equal;
xlabel(f_ax,"Local Distance from Front Wheel [m]");
ylabel(f_ax,"Previewed Displacement [m]");
title(f_ax,"Front LiDAR");
legend("Location","southwest");

% interpolate
% f_poly = polyfit(f_prev_profile(1,:),f_prev_profile(2,:),5);
% f_interplated = f_poly(1)*f_prev_profile(1,:).^5 + f_poly(2)*f_prev_profile(1,:).^4 + f_poly(3)*f_prev_profile(1,:).^3 + f_poly(4)*f_prev_profile(1,:).^2 + f_poly(5)*f_prev_profile(1,:) + f_poly(6);
% plot(f_prev_profile(1,:),f_interplated,"LineWidth",2,"Color","#ff0000","LineStyle","--");

xlim([range_min,range_max]);
% axis equal
ylim([-0.1,0.1]);
xlabel("\itX \rm[m]");
ylabel("\itZ \rm[m]");
grid on
fontname(gcf,"Times New Roman");
fontsize(gca,10,"points");

% rear
r_line = r_ospc.Location(r_ospc.Location(:,2)>=p_min & r_ospc.Location(:,2)<=p_max & r_ospc.Location(:,1)<=range_max & r_ospc.Location(:,1)>=range_min,:,:);
% r_line = r_ospc.Location(r_ospc.Location(:,1)>=-0.075 & r_ospc.Location(:,1)<=0.075 & r_ospc.Location(:,2)<=7 & r_ospc.Location(:,2)>=5.06,:,:);
[~,r_ind] = sort(r_line(:,1));
r_prev_profile=r_line(r_ind,[true false true])';

r_ax = subplot(212);
r_sc = scatter(r_prev_profile(1,:),r_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#0000ff","DisplayName","Raw Data"); hold on; % picked up points
r_correct_road = plot(r_dis_total,road_total,"LineWidth",2,"Color","#aaaaaa","DisplayName","Acutual Road");
r_pl = plot(r_prev_profile(1,:),movmean(r_prev_profile(2,:),mean_data_num),"LineWidth",2,"LineStyle",":","Color","#ff0000","DisplayName","Moving Average"); % moving average
grid on;
xlim(r_ax,[range_min,range_max]);
ylim(r_ax,[-0.08, 0.08]);
% axis equal;
xlabel(r_ax,"Local Distance from Front Wheel [m]");
ylabel(r_ax,"Previewed Displacement [m]");
title(r_ax,"Roof LiDAR");
legend("Location","southwest");

xlim([range_min,range_max]);
ylim([-0.1,0.1]);
xlabel("\itX \rm[m]");
ylabel("\itZ \rm[m]");
grid on
fontname(gcf,"Times New Roman");
fontsize(gca,10,"points");


%% calculate Error between actual road and estimated road
% front
f_dis_total_p = [f_dis_total, f_prev_profile(1,:)];
[f_dis_total_p,f_dis_idx] = sort(f_dis_total_p);
f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);

f_error = [f_prev_profile(1,:); movmean(f_prev_profile(2,:),mean_data_num) - f_correct_road_p(ismember(f_dis_total_p, f_prev_profile(1,:)))];

% roof
r_dis_total_p = [r_dis_total, r_prev_profile(1,:)];
[r_dis_total_p,r_dis_idx] = sort(r_dis_total_p);
r_correct_road_p = interp1(r_dis_total,road_total,r_dis_total_p);

r_error = [r_prev_profile(1,:); movmean(r_prev_profile(2,:),mean_data_num) - r_correct_road_p(ismember(r_dis_total_p, r_prev_profile(1,:)))];
