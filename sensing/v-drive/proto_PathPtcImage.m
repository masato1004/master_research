close all;

%% configuration
use_path = true;
use_ptc = true;
use_img = true;

animation = false;

%% define pcd and img
lidar_name = "roof"

pcd_dir_name = "ply_"+lidar_name+"-lidar";
img_dir_name = "image";

pcd_list = dir(pcd_dir_name+"/*.pcd");
img_list = dir(img_dir_name+"/*.png");

save_dir_name = "imageNdepth";
if ~exist(save_dir_name,'dir')
    mkdir(save_dir_name)
end

%% define ego csv and load positions
filename = "scenario/scenario_1_divp_Veh_NissanXtrail_1.csv";
opts = detectImportOptions(filename);
opts.SelectedVariableNames = ["timestamp", "pos_x","pos_y","pos_z", "yaw_rad", "pitch_rad", "roll_rad"];
pos_table = readtable(filename,opts);

%% video setting
sampling_freq = 20;
sampling_period = 1/sampling_freq;

if animation
    videoname = save_dir_name+"/path_with_camera";%+lidar_name;
    video = VideoWriter(videoname,'MPEG-4');
    video.FrameRate = sampling_freq;
    open(video);
end

%% camera parameter
img_w = 3840;
img_h = 2160;
k1 = 1.36648;
k2 = 1.79417;
k3 = 0.1704;
k4 = 1.90693;
k5 = 2.64875;
k6 = 0.97058;
p1 = 0.00014;
p2 = -0.00008;
fx = 2445.66438;
fy = 2444.75377;
cx = 1905.44853;
cy = 1073.60153;
imageSize = [img_h, img_w];
focalLength      = [fx, fy];
principalPoint   = [cx, cy];
RadialDistortion6 = [k1, k2, k3, k4, k5, k6];
TangentialDistortion = [p1,p2];
% intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",RadialDistortion,"TangentialDistortion",TangentialDistortion);

%% cam2roof-lidar transformation
lidar_position_r = [1.110943, 0., 1.998877];
lidar_angles_r = [0., 0., 0.];
lidar_position_f = [3.639492, 0., 0.662594];
lidar_angles_f = [0., deg2rad(22.500000), 0.];

if lidar_name == "front"
    lidar_position = lidar_position_f;
    lidar_angles   =   lidar_angles_f;
elseif lidar_name == "roof"
    lidar_position = lidar_position_r;
    lidar_angles   =   lidar_angles_r;
end
camera_position = [1.690000, 0.0, 1.500000];
% camera_position = [1.881159, 0.0, 1.554000];
camera_angles = [0., 0., 0.];

%% figure
img_fig = figure('Position',[300,300,865.67,524.67]);


imax = length(pcd_list);
strlen = 0;
for i = 1:1:length(pcd_list)
    pcd_name = pcd_list(i).name;
    img_name = img_list(i).name;
    
    %% load img
    img = imread(img_dir_name+"/"+img_name);
    
    %% imshow
    if use_img
        imshow(img);
    end

    %% load path view
    if use_path
        pos_idx = abs(pos_table.timestamp - (i-1)*sampling_period) < 1e-10;
        pos = [pos_table.pos_x(pos_idx),pos_table.pos_y(pos_idx),pos_table.pos_z(pos_idx)];
        agl = [pos_table.yaw_rad(pos_idx),pos_table.pitch_rad(pos_idx),pos_table.roll_rad(pos_idx)];
        ptc_path_from_pos = pointCloud([pos_table.pos_x ,pos_table.pos_y ,pos_table.pos_z ] - pos);
        
        R_pos = eul2rotm(-agl);
        A_pos = [[R_pos;0,0,0],[0;0;0; 1]];
        A_pos2cam = [[eye(3);0,0,0], [(-camera_position)'; 1]];
        pos_tform = rigidtform3d(A_pos);
        pos_tform2 = rigidtform3d(A_pos2cam);
        ptc_path_from_car = pctransform(ptc_path_from_pos,pos_tform);
        front_idx = ptc_path_from_car.Location(:,1)>0;
        
        [path_depth,path_camerapoints] = func_projectLidarToDepthImage(ptc_path_from_car, A_pos2cam, focalLength, principalPoint, imageSize, RadialDistortion6, TangentialDistortion);
    end
    
    if i > 1
        %% load pcd
        if use_ptc
            ptCloud = pcread(pcd_dir_name+"/"+pcd_name);
            idx = sum(ptCloud.Location(:,:) ~= 0,2)~=0;
            ptCloud = pointCloud(ptCloud.Location(idx,:));
        
            %% pcd on img
            eulerAngle1 = lidar_angles;
            translation =  - camera_position + lidar_position;
            R1 = eul2rotm(eulerAngle1);
            A1 = [[R1;0,0,0],[translation';1]];
            % ptCloud = pctransform(ptCloud,rigidtform3d(A1));
            
            [depth,lidar_camerapoints] = func_projectLidarToDepthImage(ptCloud, A1, focalLength, principalPoint, imageSize, RadialDistortion6, TangentialDistortion);
        end

        %% show depth onto img
        if use_ptc && use_img
            hold on
            scatter(depth(:,1),depth(:,2),5,depth(:,3),'MarkerEdgeColor','flat','MarkerFaceColor','flat')
            colormap(turbo);
            clim([0 100])
            hold off
        end

        %% visualize BBox of the ego vehicle
        if use_ptc && ~use_img
            % view1
            subplot(2,2,1)
            lidar = scatter3(ptCloud.Location(:,1)+camera_position(1),ptCloud.Location(:,2),ptCloud.Location(:,3)+camera_position(3),2,ptCloud.Location(:,3),'fill');
            hold on
            roi1 = drawcuboid(lidar,'Color','r','Position',[-1 -1.793/2 0 4.642 1.793 1.748]);
            if use_path
                scatter3(ptc_path_from_car.Location(front_idx,1),ptc_path_from_car.Location(front_idx,2),ptc_path_from_car.Location(front_idx,3),3,'red','filled')
            end
            axis equal;
            xlim([-10 40])
            ylim([-15 15])
            zlim([-2 4])
            view([-50 -10 20])
            xlabel( '\itX \rm[m]'); ylabel( '\itY \rm[m]'); zlabel( '\itZ \rm[m]');
            hold off
            
            % view2
            subplot(2,2,2)
            lidar2 = scatter3(ptCloud.Location(:,1)+camera_position(1),ptCloud.Location(:,2),ptCloud.Location(:,3)+camera_position(3),2,ptCloud.Location(:,3),'fill');
            hold on
            roi2 = drawcuboid(lidar2,'Color','r','Position',[-1 -1.793/2 0 4.642 1.793 1.748]);
            if use_path
                scatter3(ptc_path_from_car.Location(front_idx,1),ptc_path_from_car.Location(front_idx,2),ptc_path_from_car.Location(front_idx,3),3,'red','filled')
            end
            axis equal;
            xlim([-5 20])
            ylim([-15 15])
            zlim([-2 4])
            view([-90 90])
            xlabel( '\itX \rm[m]'); ylabel( '\itY \rm[m]'); zlabel( '\itZ \rm[m]');
            hold off
            
            % view3
            subplot(2,1,2)
            lidar3 = scatter3(ptCloud.Location(:,1)+camera_position(1),ptCloud.Location(:,2),ptCloud.Location(:,3)+camera_position(3),2,ptCloud.Location(:,3),'fill');
            hold on
            roi3 = drawcuboid(lidar3,'Color','r','Position',[-1 -1.793/2 0 4.642 1.793 1.748]);
            if use_path
                scatter3(ptc_path_from_car.Location(front_idx,1),ptc_path_from_car.Location(front_idx,2),ptc_path_from_car.Location(front_idx,3),3,'red','filled')
            end
            axis equal;
            xlim([-10 40])
            ylim([-20 20])
            zlim([-2 4])
            view([0 0])
            xlabel( '\itX \rm[m]'); ylabel( '\itY \rm[m]'); zlabel( '\itZ \rm[m]');
            hold off

            fontname(gcf,"Arial");
            fontsize(img_fig().Children,13,"points");
        end
    end

    %% visualize path on RGB image
    if use_path && use_img
        hold on
        scatter(path_depth(:,1),path_depth(:,2),5,path_depth(:,3),'MarkerEdgeColor','flat','MarkerFaceColor','flat')
        colormap(turbo);
        clim([0 100])
        hold off
    end

    drawnow;
    
    if animation
        frame = getframe(img_fig);
        writeVideo(video,frame);
    end

    Tmp = {'Progress: %3d/%d\n', i, imax};
    Tmp{1} = [ repmat(sprintf('\b'),[1 strlen]),  Tmp{1} ];

    Txt = sprintf(Tmp{1:3});
    fprintf(Txt);
    strlen = length(Txt) - strlen;
end
if animation
    close(video)
end