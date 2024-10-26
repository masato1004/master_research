%% read datas
% close all;
function upsampled_road = func__depth2ptc(upsampled_depth)

    depthImage_read = upsampled_depth;

    %% create original size images for pcfromdepth as new ones

    depthImage_original_size=uint16(zeros(1080, 1920));

    % for height 264
    % start_x = 353-1;
    % start_y = 449-1;
    % rect_width = 1216-1;
    % rect_height = 264-1;

    % for height 352
    start_x = 353-1;
    start_y = 449-1;
    rect_width = 1216-1;
    rect_height = 352-1;

    % for height 150
    % start_x = 503-1;
    % start_y = 449-1;
    % rect_width = 1216-1;
    % rect_height = 150-1;

    depthImage_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width) = depthImage_read;
    depthImage = depthImage_original_size;

    %% pc from depth
    focalLength      = [1070.6, 1070.58];
    principalPoint   = [964.41, 547.833];
    imageSize        = size(depthImage_original_size,[1,2]);  % 352 1216
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

    ptCloud = pcfromdepth(depthImage,depthScaleFactor,intrinsics);
    tform = rigidtform3d([-90 0 -90],[0 0 0]);
    ptCloud = pctransform(ptCloud,tform);
    % pcshow(ptCloud);

    pcin=pointCloud(reshape(ptCloud.Location,[],3));
    % downptCloud = pcdownsample(groundtruthptCloud,'gridAverage',0.05);
    % downptCloud = pcdownsample(rawptCloud,'gridAverage',0.05);
    % downptCloud = pcdownsample(ptCloud,'gridAverage',0.1);
    % downptCloud = pointCloud(downptCloud.Location(downptCloud.Location(:,1)<7&downptCloud.Location(:,1)>0&downptCloud.Location(:,2)<3&downptCloud.Location(:,2)>-3,:,:));
    downpc = pcin;
    eliminate_idx = downpc.Location(:,1)<9&downpc.Location(:,1)>2.5&downpc.Location(:,2)<2&downpc.Location(:,2)>-2;
    downptCloud = pointCloud(downpc.Location(eliminate_idx,:,:));
    % downptCloud = pcdownsample(downptCloud,'gridAverage',0.001);

    pcin_eliminate_idx = pcin.Location(:,1)>0.5;
    pcin = pointCloud(pcin.Location(pcin_eliminate_idx,:,:));

    [ptCloud, plaen_mesh, plane_tform] = fitplane(pcin,downptCloud,0.004);
    ptCloud = pctransform(ptCloud,r_tform_cam2wheel);
    upsampled_road = ptCloud;
end

% % ptloc=ptCloud.Location;
% % ptloc(ptloc(:,1)<0,1)=2;
% % ptloc(ptloc(:,3)>0.5,3)=0;
% % ptCloud=pointCloud(ptloc);
% % colorImage_new = reshape(colorImage,[],3);
% temp_fig = figure("Position",[100,100,250,200]);
% % pcshow(reshape(ptCloud.Location,[],3));
% % pcshow(reshape(ptCloud.Location,[],3),reshape(colorImage,[],3));
% pcshow(ptCloud);
% % ptCloud=ptCloud_new;
% xlabel("\itX \rm[m]");
% ylabel("\itY \rm[m]");
% zlabel("\itZ \rm[m]");
% fontname(gcf,"Times New Roman");
% fontsize(gca,9,"points");
% set(gcf,'color','w');
% set(gca,'color','w');
% set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
% saveas(temp_fig, "test.png")

% %% set infomations
% msg_nums = [2748 2750];
% gridStep = 0.05;

% % install params
% f_ousmsg_num = msg_nums(1);
% r_ousmsg_num = msg_nums(2);

% % movmean setting
% mean_data_num = [20, 20]; % 60, 60
% % mean_data_num = [2000, 2000]; % 60, 60

% %% correct road surface profile
% max_z0 = 0.025;                                                                % [m] max road displacement
% ld = [0.05 0.15 0.05];
% start_disturbance = 3; % 2.96                                                                  % amplitude
% max_distance = 30;                                                           % [m] driving mileage
% f_dis_total = [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
% r_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
% road_total = [0,0,max_z0,max_z0,0,0];  % converting front disturbance and buffer ([m])

% %% 2d profile
% figure("Position", [100 50 260 145]);
% % figure("Position", [100 50 260 340/2]);
% range_min = 0;        % minimum measurable distance [m]
% range_max = 8;        % maximum measurable distance [m]
% pick_up_width = 1;  % width of datas for a road profile [m]
% pick_up_center = 0;   % center of pick up position [m]
% p_min = pick_up_center - pick_up_width/2;
% p_max = pick_up_center + pick_up_width/2;

% % raw
% raw_ospc = rawptCloud;
% raw_line = raw_ospc.Location(raw_ospc.Location(:,2)>=p_min & raw_ospc.Location(:,2)<=p_max & raw_ospc.Location(:,1)<=range_max & raw_ospc.Location(:,1)>=range_min,:,:);
% % raw_line = raw_ospc.Location(raw_ospc.Location(:,1)>=-0.075 & raw_ospc.Location(:,1)<=0.075 & raw_ospc.Location(:,2)<=7 & raw_ospc.Location(:,2)>=5.06,:,:);
% [~,raw_ind] = sort(raw_line(:,1));
% raw_prev_profile=raw_line(raw_ind,[true false true])';
% raw_dis_total_p = [f_dis_total, raw_prev_profile(1,:)];
% [raw_dis_total_p,~] = sort(raw_dis_total_p);
% raw_correct_road_p = interp1(f_dis_total,road_total,raw_dis_total_p);
% % raw_sc = scatter(raw_prev_profile(1,:),raw_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#00ff00","DisplayName","Raw Data"); hold on;  % picked up points
% % raw_correct_road = plot(raw_dis_total_p,raw_correct_road_p,"LineWidth",2,"Color","#aaaaaa","DisplayName","Actual Road"); hold on;
% % raw_pl = plot(raw_prev_profile(1,:),movmean(raw_prev_profile(2,:),mean_data_num),"LineWidth",2,"LineStyle",":","Color","#ff0000","DisplayName","Moving Average"); % moving average

% % gt
% gt_ospc = gtptCloud;
% gt_line = gt_ospc.Location(gt_ospc.Location(:,2)>=p_min & gt_ospc.Location(:,2)<=p_max & gt_ospc.Location(:,1)<=range_max & gt_ospc.Location(:,1)>=range_min,:,:);
% % gt_line = gt_ospc.Location(gt_ospc.Location(:,1)>=-0.075 & gt_ospc.Location(:,1)<=0.075 & gt_ospc.Location(:,2)<=7 & gt_ospc.Location(:,2)>=5.06,:,:);
% [~,gt_ind] = sort(gt_line(:,1));
% gt_prev_profile=gt_line(gt_ind,[true false true])';
% gt_dis_total_p = [f_dis_total, gt_prev_profile(1,:)];
% [gt_dis_total_p,~] = sort(gt_dis_total_p);
% gt_correct_road_p = interp1(f_dis_total,road_total,gt_dis_total_p);
% % gt_sc = scatter(gt_prev_profile(1,:),gt_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#00ff00","DisplayName","Ground Truth Data"); hold on;  % picked up points
% % gt_pl = plot(gt_prev_profile(1,:),movmean(gt_prev_profile(2,:),mean_data_num),"LineWidth",2,"LineStyle",":","Color","#ff0000","DisplayName","Moving Average"); % moving average

% % prediction
% f_ospc = ptCloud;
% f_line = f_ospc.Location(f_ospc.Location(:,2)>=p_min & f_ospc.Location(:,2)<=p_max & f_ospc.Location(:,1)<=range_max & f_ospc.Location(:,1)>=range_min,:,:);
% % f_line = f_ospc.Location(f_ospc.Location(:,1)>=-0.075 & f_ospc.Location(:,1)<=0.075 & f_ospc.Location(:,2)<=7 & f_ospc.Location(:,2)>=5.06,:,:);
% [~,f_ind] = sort(f_line(:,1));
% f_prev_profile=f_line(f_ind,[true false true])';
% f_dis_total_p = [f_dis_total, f_prev_profile(1,:)];
% [f_dis_total_p,~] = sort(f_dis_total_p);
% f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);
% f_sc = scatter(f_prev_profile(1,:),f_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#0000ff","DisplayName","Predicted Data"); hold on;  % picked up points
% % raw_sc = scatter(raw_prev_profile(1,:),raw_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#00aa00","DisplayName","Raw Data"); hold on;  % picked up points
% f_correct_road = plot(f_dis_total_p,f_correct_road_p,"LineWidth",2,"Color","#aaaaaa","DisplayName","Actual Road"); hold on;
% f_pl = plot(f_prev_profile(1,:),movmean(f_prev_profile(2,:),mean_data_num),"LineWidth",2,"LineStyle",":","Color","#ff0000","DisplayName","Moving Average"); % moving average

% grid on;
% xlim([range_min,range_max]);
% ylim([-0.08, 0.08]);
% % axis equal;
% xlabel("Local Distance from Front Wheel [m]");
% ylabel("Previewed Displacement [m]");
% % title("Front LiDAR");
% legend("Location","southwest");
% % interpolate
% % f_poly = polyfit(f_prev_profile(1,:),f_prev_profile(2,:),5);
% % f_interplated = f_poly(1)*f_prev_profile(1,:).^5 + f_poly(2)*f_prev_profile(1,:).^4 + f_poly(3)*f_prev_profile(1,:).^3 + f_poly(4)*f_prev_profile(1,:).^2 + f_poly(5)*f_prev_profile(1,:) + f_poly(6);
% % plot(f_prev_profile(1,:),f_interplated,"LineWidth",2,"Color","#ff0000","LineStyle","--");
% xlim([range_min,range_max]);
% % axis equal
% ylim([-0.1,0.05]);
% xlabel("\itX \rm[m]");
% ylabel("\itZ \rm[m]");
% grid on
% fontname(gcf,"Times New Roman");
% fontsize(gca,9,"points");

% %% calculate Error between actual road and estimated road
% % front
% f_dis_total_p = [f_dis_total, f_prev_profile(1,:)];
% [f_dis_total_p,f_dis_idx] = sort(f_dis_total_p);
% f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);
% f_correct_prev = [f_prev_profile(1,:); f_correct_road_p(ismember(f_dis_total_p, f_prev_profile(1,:)))];
% f_disturbance = f_correct_prev(2,f_correct_prev(1,:)<start_disturbance+sum(ld) & f_correct_prev(1,:)>start_disturbance);
% f_prev_movmean = movmean(f_prev_profile(2,:),mean_data_num);

% f_error = [f_prev_profile(1,:); movmean(f_prev_profile(2,:),mean_data_num) - f_correct_road_p(ismember(f_dis_total_p, f_prev_profile(1,:)))];
% MAE_on_2dRPF = double(mean(abs(f_error(2,:))))
% RMSE_on_2dRPF = double(sqrt(mean(f_error(2,:).^2)))
% RMSE_on_bump = double(rmse(f_prev_movmean(f_prev_profile(1,:)<start_disturbance+sum(ld) & f_prev_profile(1,:)>start_disturbance) , f_disturbance))
