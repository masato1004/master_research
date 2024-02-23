%% set infomations
msg_nums = [966 969];

% install params
f_ousmsg_num = msg_nums(1);
r_ousmsg_num = msg_nums(2);

%% translat with position parameter from cad
rotate_angle_cam2wheel = [0 0 0];
translation_cam2wheel = [-0.9701 0 0]; % from cad
tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,translation_cam2wheel);

%% front point Cloud
figure
f_ospc_read = readXYZ(f_ousMsgs{f_ousmsg_num}); % 965 -> hump
f_ospc = pointCloud(f_ospc_read);
f_ospc = pctransform(f_ospc,f_tform);
f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,1)>=2.1 & f_ospc.Location(:,2)>=-2 & f_ospc.Location(:,2)<=2,:,:));
[f_ospc, ~, plane_tform] = fitplane(f_ospc,0.01);
f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,3)>=-1 & f_ospc.Location(:,3)<=0.2 & f_ospc.Location(:,1)<=8.5,:,:));
f_ospc = pctransform(f_ospc,tform_cam2wheel);
f_ouspc_show = pcshow(f_ospc);
xlabel("\itX \rm[m]");
ylabel("\itY \rm[m]");
zlabel("\itZ \rm[m]");
fontname(gcf,"Times New Roman");
fontsize(gca,16,"points");

%% rear point Cloud
figure
r_ospc_read = readXYZ(r_ousMsgs{r_ousmsg_num}); % 965 -> hump
r_ospc = pointCloud(r_ospc_read);
r_ospc = pctransform(r_ospc,r_tform);
r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,1)>=1.5 & r_ospc.Location(:,2)>=-2 & r_ospc.Location(:,2)<=2,:,:));
[r_ospc, ~, plane_tform] = fitplane(r_ospc,0.01);
r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,3)>=-1 & r_ospc.Location(:,3)<=0.2 & r_ospc.Location(:,1)<=8.5,:,:));
r_ospc = pctransform(r_ospc,tform_cam2wheel);
r_ouspc_show = pcshow(r_ospc);
xlabel("\itX \rm[m]");
ylabel("\itY \rm[m]");
zlabel("\itZ \rm[m]");
fontname(gcf,"Times New Roman");
fontsize(gca,16,"points");

%% PICK UP AS 2D
range_min = 0;        % minimum measurable distance [m]
range_max = 7;        % maximum measurable distance [m]
pick_up_width = 0.3;  % width of datas for a road profile [m]
pick_up_center = 0;   % center of pick up position [m]

p_min = pick_up_center - pick_up_width/2;
p_max = pick_up_center + pick_up_width/2;

% front
f_line = f_ospc.Location(f_ospc.Location(:,2)>=p_min & f_ospc.Location(:,2)<=p_max & f_ospc.Location(:,1)<=range_max & f_ospc.Location(:,1)>=range_min,:,:);
% f_line = f_ospc.Location(f_ospc.Location(:,1)>=-0.075 & f_ospc.Location(:,1)<=0.075 & f_ospc.Location(:,2)<=7 & f_ospc.Location(:,2)>=5.06,:,:);
[~,f_ind] = sort(f_line(:,1));
f_prev_profile=f_line(f_ind,[true false true])';

figure('name', "RSP from front-LiDAR",'Position', [500-20 500-20 1400 380]);
scatter(f_prev_profile(1,:),f_prev_profile(2,:),1,"Color","#0000ff"); hold on;

% moving average
plot(f_prev_profile(1,:),movmean(f_prev_profile(2,:),40),"LineWidth",2,"Color","#ff0000");

% interpolate
% f_poly = polyfit(f_prev_profile(1,:),f_prev_profile(2,:),5);
% f_interplated = f_poly(1)*f_prev_profile(1,:).^5 + f_poly(2)*f_prev_profile(1,:).^4 + f_poly(3)*f_prev_profile(1,:).^3 + f_poly(4)*f_prev_profile(1,:).^2 + f_poly(5)*f_prev_profile(1,:) + f_poly(6);
% plot(f_prev_profile(1,:),f_interplated,"LineWidth",2,"Color","#ff0000","LineStyle","--");

fontname(gcf,"Times New Roman");
xlim([0,7]);
% axis equal
ylim([-0.1,0.1]);
xlabel("\itX \rm[m]");
ylabel("\itZ \rm[m]");
grid on
fontsize(gca,16,"points");

% rear
r_line = r_ospc.Location(r_ospc.Location(:,2)>=p_min & r_ospc.Location(:,2)<=p_max & r_ospc.Location(:,1)<=range_max & r_ospc.Location(:,1)>=range_min,:,:);
% r_line = r_ospc.Location(r_ospc.Location(:,1)>=-0.075 & r_ospc.Location(:,1)<=0.075 & r_ospc.Location(:,2)<=7 & r_ospc.Location(:,2)>=5.06,:,:);
[~,r_ind] = sort(r_line(:,1));
r_prev_profile=r_line(r_ind,[true false true])';

figure('name', "RSP from roof-LiDAR",'Position', [500-40 500-20 1400 380]);
scatter(r_prev_profile(1,:),r_prev_profile(2,:),1,"Color","#0000ff")
fontname(gcf,"Times New Roman");
xlim([0,7]);
ylim([-0.1,0.1]);
xlabel("\itX \rm[m]");
ylabel("\itZ \rm[m]");
grid on
fontsize(gca,16,"points");