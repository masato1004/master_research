list_depth_imgs = dir("depth_imgs/*.png");
list_color_imgs = dir("color_imgs/*.png");

file_name = "1708413010.220381.png";
depthImage_read = imread("depth_imgs/"+list_depth_imgs(1).name);
colorImage_read = imread("color_imgs/"+list_color_imgs(1).name);

%% create original size images for pcfromdepth as new ones
depthImage_original_size=uint16(zeros(1080, 1920));
colorImage_original_size=uint8(ones(1080, 1920, 3));

start_x = 353-1;
start_y = 449-1;
rect_width = 1216-1;
rect_height = 352-1;
depthImage_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width) = depthImage_read;
colorImage_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width,:) = colorImage_read;
depthImage = depthImage_original_size;
colorImage = colorImage_original_size;

%% pc from depth
focalLength      = [1070.6, 1070.58];
principalPoint   = [964.41, 547.833];
imageSize        = size(colorImage,[1,2]);  % 352 1216
% imageSize        = [1080, 1920];
RadialDistortion = [-0.0546184,0.0269859,-0.0105149];
TangentialDistortion = [3.24293e-05,-0.000156357];
intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",RadialDistortion,"TangentialDistortion",TangentialDistortion);
depthScaleFactor = 65535/15; % Z = 深度画像[u,v]/スケールファクタ
maxCameraDepth   = 15;

%% translat with position parameter from cad
rotate_angle_cam2wheel = [0 0 0];
f_translation_cam2wheel = [0 1 0]; % from cad
r_translation_cam2wheel = [-1 0 0]; % from cad
f_tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,f_translation_cam2wheel);
r_tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,r_translation_cam2wheel);

ptCloud = pcfromdepth(depthImage,depthScaleFactor,intrinsics,ColorImage=colorImage);
tform = rigidtform3d([-90 0 -90],[0 0 0]);
ptCloud = pctransform(ptCloud,tform);
pcshow(ptCloud);

pcin=pointCloud(reshape(ptCloud.Location,[],3));
downptCloud = pcdownsample(ptCloud,'gridAverage',0.05);
downptCloud = pointCloud(downptCloud.Location(downptCloud.Location(:,1)<6,:,:));
[ptCloud, plaen_mesh, plane_tform] = fitplane(pcin,downptCloud,0.01);
ptCloud = pctransform(ptCloud,r_tform_cam2wheel);
pcshow(reshape(ptCloud.Location,[],3),reshape(colorImage,[],3));

%% set infomations
msg_nums = [2748 2750];
gridStep = 0.05;

% install params
f_ousmsg_num = msg_nums(1);
r_ousmsg_num = msg_nums(2);

% movmean setting
mean_data_num = [400, 400];

%% correct road surface profile
max_z0 = 0.025;                                                                % [m] max road displacement
ld = [0.05 0.15 0.05];
start_disturbance = 2.96;                                                                  % amplitude
max_distance = 30;                                                           % [m] driving mileage
f_dis_total = [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
r_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
road_total = [0,0,max_z0,max_z0,0,0];  % converting front disturbance and buffer ([m])

%% 2d profile
figure("Position", [100 50 260 340]);
range_min = 0;        % minimum measurable distance [m]
range_max = 8;        % maximum measurable distance [m]
pick_up_width = 1;  % width of datas for a road profile [m]
pick_up_center = 0;   % center of pick up position [m]
p_min = pick_up_center - pick_up_width/2;
p_max = pick_up_center + pick_up_width/2;
% front
f_ospc = ptCloud;
f_line = f_ospc.Location(f_ospc.Location(:,2)>=p_min & f_ospc.Location(:,2)<=p_max & f_ospc.Location(:,1)<=range_max & f_ospc.Location(:,1)>=range_min,:,:);
% f_line = f_ospc.Location(f_ospc.Location(:,1)>=-0.075 & f_ospc.Location(:,1)<=0.075 & f_ospc.Location(:,2)<=7 & f_ospc.Location(:,2)>=5.06,:,:);
[~,f_ind] = sort(f_line(:,1));
f_prev_profile=f_line(f_ind,[true false true])';
f_dis_total_p = [f_dis_total, f_prev_profile(1,:)];
[f_dis_total_p,~] = sort(f_dis_total_p);
f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);
f_sc = scatter(f_prev_profile(1,:),f_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#0000ff","DisplayName","Raw Data"); hold on;  % picked up points
f_correct_road = plot(f_dis_total_p,f_correct_road_p,"LineWidth",2,"Color","#aaaaaa","DisplayName","Actual Road"); hold on;
f_pl = plot(f_prev_profile(1,:),movmean(f_prev_profile(2,:),mean_data_num),"LineWidth",2,"LineStyle",":","Color","#ff0000","DisplayName","Moving Average"); % moving average
grid on;
xlim([range_min,range_max]);
ylim([-0.08, 0.08]);
% axis equal;
xlabel("Local Distance from Front Wheel [m]");
ylabel("Previewed Displacement [m]");
title("Front LiDAR");
legend("Location","southwest");
% interpolate
% f_poly = polyfit(f_prev_profile(1,:),f_prev_profile(2,:),5);
% f_interplated = f_poly(1)*f_prev_profile(1,:).^5 + f_poly(2)*f_prev_profile(1,:).^4 + f_poly(3)*f_prev_profile(1,:).^3 + f_poly(4)*f_prev_profile(1,:).^2 + f_poly(5)*f_prev_profile(1,:) + f_poly(6);
% plot(f_prev_profile(1,:),f_interplated,"LineWidth",2,"Color","#ff0000","LineStyle","--");
xlim([range_min,range_max]);
% axis equal
ylim([-0.1,0.05]);
xlabel("\itX \rm[m]");
ylabel("\itZ \rm[m]");
grid on
fontname(gcf,"Times New Roman");
fontsize(gca,10,"points");