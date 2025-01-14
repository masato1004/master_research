function [dpcd] = F_depth2gradient(depthImage_read,cam_params,dc_params,angle)
    %% create original size images for pcfromdepth as new ones
    img_w = dc_params.original_img_w;
    img_h = dc_params.original_img_h;
    depthImage_original_size=uint16(zeros(img_h, img_w));

    % crop_info = {dc_params.start_y:dc_params.start_y+dc_params.rect_height;dc_params.start_x:dc_params.start_x+dc_params.rect_width};
    
    depthImage_original_size(dc_params.start_y:dc_params.start_y+dc_params.rect_height,dc_params.start_x:dc_params.start_x+dc_params.rect_width) = depthImage_read;
    depthImage = depthImage_original_size;

    %% camera parameter
    depthScaleFactor = 65535/dc_params.maxCameraDepth; % Z = 深度画像[u,v]/スケールファクタ


    %% translat with position parameter from cad
    rotate_angle_cam2wheel = [0 0 0];
    f_translation_cam2wheel = [0 1 0]; % from cad
    r_translation_cam2wheel = [-1 0 0]; % from cad
    f_tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,f_translation_cam2wheel);
    r_tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,r_translation_cam2wheel);

    % ptCloud = pcfromdepth(depthImage,depthScaleFactor,intrinsics,ColorImage=colorImage);
    [ptCloud,~,~,dpcd] = F_projectDepthImageToLidar(depthImage,cam_params.focalLength,cam_params.principalPoint,cam_params.RadialDistortion6,cam_params.TangentialDistortion,depthScaleFactor,dc_params.crop_info);
    tform = rigidtform3d([-90 0 -90],cam_params.camera_position);
    dpcd = pctransform(dpcd,tform);
    R_pos = eul2rotm(-angle);
    A_pos = [[R_pos;0,0,0],[0;0;0; 1]];
    pos_tform = rigidtform3d(A_pos);
    dpcd = pctransform(dpcd,pos_tform);
    % pcshow(ptCloud);

    max_x = dc_params.maxCameraDepth;
    dpcd_eliminate_idx = dpcd.Location(:,1)>1.9&dpcd.Location(:,1)<max_x&dpcd.Location(:,2)>-2&dpcd.Location(:,2)<2;
    dpcd = pointCloud(dpcd.Location(dpcd_eliminate_idx,:,:));
    dpcd = pointCloud([dpcd.Location(:,1),dpcd.Location(:,2),-dpcd.Location(:,3) + mean(dpcd.Location(:,3))]);
    
    % [~,~,outlierIndices] = pcfitplane(dpcd,0.002);
    % gradient = dpcd.Location(outlierIndices,:);

    % ptloc=ptCloud.Location;
    % ptloc(ptloc(:,1)<0,1)=2;
    % ptloc(ptloc(:,3)>0.5,3)=0;
    % ptCloud=pointCloud(ptloc);
    % colorImage_new = reshape(colorImage,[],3);

    % figure();
    % pcshow(ptCloud);
    % axis equal;
    % fontname(gcf,"Arial");
    % fontsize(gca,8,"points");
    % set(gcf,'color','w');
    % set(gca,'color','w');
    % set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
    % figure();
    % pcshow(ptCloud.Location);
    % fontname(gcf,"Arial");
    % fontsize(gca,8,"points");
    % set(gcf,'color','w');
    % set(gca,'color','w');
    % set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
    % clim([-0.1 0.1])
    % % temp_fig = figure("Position",[100,100,150,120]);
    % temp_fig = figure();
    % diffcolor = diffcolor(ptCloud_eliminate_idx);
    % % diffcolor(diffcolor>0.003)=0;
    % % pcshow(pointCloud(ptCloud.Location,Color=repmat(diffImage(ptCloud_eliminate_idx),[1,3])./max(diffImage(ptCloud_eliminate_idx))));
    % scatter(ptCloud.Location(:,1),ptCloud.Location(:,2),3,diffcolor,"filled");
    % colormap("jet")
    % % clim([-0.01 0.01])
    % % clim([-0.0000001 0.0000001])
    % % pcshow(reshape(ptCloud.Location,[],3),reshape(colorImage,[],3));
    % % pcshow(ptCloud.Location);
    % % ptCloud=ptCloud_new;
    % xlabel("\itX \rm[m]");
    % ylabel("\itY \rm[m]");
    % % zlabel("\itZ \rm[m]");
    % clim([-0.002 0.002])
    % xlim([min(ptCloud.Location(:,1)),max(ptCloud.Location(:,1))]);
    % ylim([min(ptCloud.Location(:,2)),max(ptCloud.Location(:,2))]);
    % axis equal;
    % fontname(gcf,"Arial");
    % fontsize(gca,8,"points");
    % set(gcf,'color','w');
    % set(gca,'color','w');
    % set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

    % figure();
    % pcshow(dpcd.Location(outlierIndices,:),MarkerSize=40)
    % xlabel("\itX \rm[m]");
    % ylabel("\itY \rm[m]");
    % zlabel("\itZ \rm[m]");
    % xlim([min(ptCloud.Location(:,1)),max(ptCloud.Location(:,1))]);
    % ylim([min(ptCloud.Location(:,2)),max(ptCloud.Location(:,2))]);
    % fontname(gcf,"Arial");
    % fontsize(gca,8,"points");
    % set(gcf,'color','w');
    % set(gca,'color','w');
    % set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
    % % saveas(temp_fig, "test.png")
    % % saveas(temp_fig,"C:\Users\INOUE MASATO\OneDrive - keio.jp\高橋研究室\journal\AutomotiveInnovation\figs\ptc_"+figname+"_"+RMSE_on_2dRPF+"_"+RMSE_on_bump+".fig")


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

    %% 2d profile
    % fig_2d = figure("Position", [100 50 300 130]);
    % % figure("Position", [100 50 260 340/2]);
    % range_min = 0;        % minimum measurable distance [m]
    % range_max = 10;        % maximum measurable distance [m]
    % pick_up_width = 0.7;  % width of datas for a road profile [m]
    % pick_up_center = 0;   % center of pick up position [m]
    % p_min = pick_up_center - pick_up_width/2;
    % p_max = pick_up_center + pick_up_width/2;
    % 
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
    % 
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
    % 
    % % prediction
    % f_ospc = ptCloud;
    % f_line = f_ospc.Location(f_ospc.Location(:,2)>=p_min & f_ospc.Location(:,2)<=p_max & f_ospc.Location(:,1)<=range_max & f_ospc.Location(:,1)>=range_min,:,:);
    % % f_line = f_ospc.Location(f_ospc.Location(:,1)>=-0.075 & f_ospc.Location(:,1)<=0.075 & f_ospc.Location(:,2)<=7 & f_ospc.Location(:,2)>=5.06,:,:);
    % [~,f_ind] = sort(f_line(:,1));
    % f_prev_profile=f_line(f_ind,[true false true])';
    % f_dis_total_p = [f_dis_total, f_prev_profile(1,:)];
    % [f_dis_total_p,~] = sort(f_dis_total_p);
    % f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);
    % f_sc = scatter(f_prev_profile(1,:),f_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#0000ff","DisplayName","Estimate"); hold on;  % picked up points
    % % raw_sc = scatter(raw_prev_profile(1,:),raw_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#00aa00","DisplayName","Raw Data"); hold on;  % picked up points
    % f_correct_road = plot(f_dis_total_p,f_correct_road_p,"LineWidth",2,"Color","#aaaaaa","DisplayName","Actual"); hold on;
    % f_pl = plot(f_prev_profile(1,:),movmean(f_prev_profile(2,:),mean_data_num),"LineWidth",2,"LineStyle",":","Color","#ff0000","DisplayName","MA"); % moving average
    % 
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
    % fontname(gcf,"Arial");
    % fontsize(gca,8,"points");
    % % saveas(fig_2d,"C:\Users\INOUE MASATO\OneDrive - keio.jp\高橋研究室\journal\AutomotiveInnovation\figs\rsp_"+figname+"_"+RMSE_on_2dRPF+"_"+RMSE_on_bump+".fig")
    % 
    % %% calculate Error between actual road and estimated road
    % % front
    % f_dis_total_p = [f_dis_total, f_prev_profile(1,:)];
    % [f_dis_total_p,f_dis_idx] = sort(f_dis_total_p);
    % f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);
    % 
    % % [uni_f_prev_profile, uni_idx] = unique(f_prev_profile(1,:));
    % % f_prev_profile = [uni_f_prev_profile; f_prev_profile(2,uni_idx)];
    % % f_correct_road_p = [f_correct_road_p, interp1(f_dis_total_p,f_correct_road_p,uni_f_prev_profile)];
    % 
    % f_correct_prev = [f_prev_profile(1,:); f_correct_road_p(ismember(f_dis_total_p, f_prev_profile(1,:)))];
    % f_disturbance = f_correct_prev(2,f_correct_prev(1,:)<start_disturbance+sum(ld) & f_correct_prev(1,:)>start_disturbance);
    % f_prev_movmean = movmean(f_prev_profile(2,:),mean_data_num);
    % 
    % f_error = [f_prev_profile(1,:); movmean(f_prev_profile(2,:),mean_data_num) - f_correct_road_p(ismember(f_dis_total_p, f_prev_profile(1,:)))];
    % MAE_on_2dRPF = double(mean(abs(f_error(2,:))))
    % RMSE_on_2dRPF = double(sqrt(mean(f_error(2,:).^2)))
    % RMSE_on_bump = double(rmse(f_prev_movmean(f_prev_profile(1,:)<start_disturbance+sum(ld) & f_prev_profile(1,:)>start_disturbance) , f_disturbance))
    % % RMSE_on_bump = double(rmse(f_prev_profile(2,f_prev_profile(1,:)<start_disturbance+sum(ld) & f_prev_profile(1,:)>start_disturbance) , f_disturbance))
end