close all;
%% set infomations
msg_nums = [949 951];
gridStep = 0.05;

% install params
f_ousmsg_num = msg_nums(1);
r_ousmsg_num = msg_nums(2);

% movmean setting
mean_data_num = [40, 40];

%% correct road surface profile
max_z0 = 0.025;                                                                % [m] max road displacement
ld = [0.05 0.15 0.05];
start_disturbance = 6.5;                                                                  % amplitude
max_distance = 30;                                                           % [m] driving mileage
f_dis_total = [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
r_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
road_total = [0,0,max_z0,max_z0,0,0];  % converting front disturbance and buffer ([m])

%% front point Cloud
f_ospc_read = readXYZ(f_ousMsgs{f_ousmsg_num}); % 965 -> hump
f_ospc = pointCloud(f_ospc_read);
f_ospc = pctransform(f_ospc,f_tform);
f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,1)>=1.2 & f_ospc.Location(:,2)>=-4 & f_ospc.Location(:,2)<=4 & f_ospc.Location(:,3)<=5,:,:));
f_downptCloud = pcdownsample(f_ospc,'gridAverage',gridStep);
[f_ospc, ~, plane_tform] = fitplane(f_ospc,f_downptCloud,0.01);
f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,3)>=-1 & f_ospc.Location(:,3)<=0.2 & f_ospc.Location(:,2)>=-2 & f_ospc.Location(:,2)<=2 & f_ospc.Location(:,1)<=8.5,:,:));
% figure
% f_ouspc_show = pcshow(f_ospc);
% xlabel("\itX \rm[m]");
% ylabel("\itY \rm[m]");
% zlabel("\itZ \rm[m]");
% fontname(gcf,"Times New Roman");
% fontsize(gca,16,"points");

%% rear point Cloud
r_ospc_read = readXYZ(r_ousMsgs{r_ousmsg_num}); % 965 -> hump
r_ospc = pointCloud(r_ospc_read);
r_ospc = pctransform(r_ospc,r_tform);
r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,1)>=1 & r_ospc.Location(:,2)>=-4 & r_ospc.Location(:,2)<=4,:,:));
r_downptCloud = pcdownsample(r_ospc,'gridAverage',gridStep);
[r_ospc, ~, plane_tform] = fitplane(r_ospc,r_downptCloud,0.01);
r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,3)>=-1 & r_ospc.Location(:,3)<=0.2 & r_ospc.Location(:,2)>=-2 & r_ospc.Location(:,2)<=2 & r_ospc.Location(:,1)<=8.5,:,:));
% figure
% r_ouspc_show = pcshow(r_ospc);
% xlabel("\itX \rm[m]");
% ylabel("\itY \rm[m]");
% zlabel("\itZ \rm[m]");
% fontname(gcf,"Times New Roman");
% fontsize(gca,16,"points");

%% PICK UP AS 2D
range_min = 0;        % minimum measurable distance [m]
range_max = 8;        % maximum measurable distance [m]
pick_up_width = 0.3;  % width of datas for a road profile [m]
pick_up_center = 0;   % center of pick up position [m]

p_min = pick_up_center - pick_up_width/2;
p_max = pick_up_center + pick_up_width/2;

% front
f_line = f_ospc.Location(f_ospc.Location(:,2)>=p_min & f_ospc.Location(:,2)<=p_max & f_ospc.Location(:,1)<=range_max & f_ospc.Location(:,1)>=range_min,:,:);
% f_line = f_ospc.Location(f_ospc.Location(:,1)>=-0.075 & f_ospc.Location(:,1)<=0.075 & f_ospc.Location(:,2)<=7 & f_ospc.Location(:,2)>=5.06,:,:);
[~,f_ind] = sort(f_line(:,1));
f_prev_profile=f_line(f_ind,[true false true])';

f_ax = subplot(211);% front
f_correct_road = plot(f_dis_total,road_total,"LineWidth",2,"Color","#aaaaaa"); hold on;
f_sc = scatter(f_prev_profile(1,:),f_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#0000ff"); hold on;  % picked up points
f_pl = plot(f_prev_profile(1,:),movmean(f_prev_profile(2,:),mean_data_num),"LineWidth",2,"Color","#ff0000"); % moving average
grid on;
xlim(f_ax,[range_min,range_max]);
ylim(f_ax,[-0.08, 0.08]);
% axis equal;
xlabel(f_ax,"Local Distance from Front Wheel [m]");
ylabel(f_ax,"Previewed Displacement [m]");
title(f_ax,"Front LiDAR");
legend(["Actual Road","Raw data","Moving Average"],"Location","southwest");

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
r_correct_road = plot(r_dis_total,road_total,"LineWidth",2,"Color","#aaaaaa"); hold on;
r_sc = scatter(r_prev_profile(1,:),r_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#0000ff"); % picked up points
r_pl = plot(r_prev_profile(1,:),movmean(r_prev_profile(2,:),mean_data_num),"LineWidth",2,"LineStyle",":","Color","#ff0000"); % moving average
grid on;
xlim(r_ax,[range_min,range_max]);
ylim(r_ax,[-0.08, 0.08]);
% axis equal;
xlabel(r_ax,"Local Distance from Front Wheel [m]");
ylabel(r_ax,"Previewed Displacement [m]");
title(r_ax,"Roof LiDAR");
legend(["Actual Road","Raw data","Moving Average"],"Location","southwest");

xlim([range_min,range_max]);
ylim([-0.1,0.1]);
xlabel("\itX \rm[m]");
ylabel("\itZ \rm[m]");
grid on
fontname(gcf,"Times New Roman");
fontsize(gca,10,"points");