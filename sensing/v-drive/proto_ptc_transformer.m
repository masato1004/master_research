close all;

%% define pcd and img
lidar_name = "front"

pcd_dir_name = "ply_"+lidar_name+"-lidar";
img_dir_name = "image";

pcd_list = dir(pcd_dir_name+"/*.pcd");
img_list = dir(img_dir_name+"/*.png");

if ~exist("imageNdepth",'dir')
    mkdir("imageNdepth")
end

%% video setting
sampling_freq = 20;
sampling_period = 1/sampling_freq;
videoname = "imageNdepth"+"/test"+lidar_name;
video = VideoWriter(videoname,'MPEG-4');
video.FrameRate = sampling_freq;
open(video);

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
RadialDistortion = [-0.06390521968364953,0.0013426527038489695,-8.135414576415314e-06];
RadialDistortion6 = [k1, k2, k3, k4, k5, k6];
TangentialDistortion = [p1,p2];
intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",RadialDistortion,"TangentialDistortion",TangentialDistortion);

%% cam2roof-lidar transformation
lidar_position_r = [1.110943, 0., 1.998877];
lidar_angles_r = [0., 0., 0.];
lidar_position_f = [3.639492, 0., 0.662594];
lidar_angles_f = [0., deg2rad(22.500000), 0.];

lidar_position = lidar_position_f;
lidar_angles   =   lidar_angles_f;
camera_position = [1.690000, 0.0, 1.500000];
camera_angles = [0., 0., 0.];

%% figure
img_fig = figure();
for i = 1:length(pcd_list)
    pcd_name = pcd_list(i).name;
    img_name = img_list(i).name;
    
    %% load img
    img = imread(img_dir_name+"/"+img_name);
    
    %% imshow
    imshow(img);
    
    if i > 1
        %% load pcd
        ptCloud = pcread(pcd_dir_name+"/"+pcd_name);
        idx = sum(ptCloud.Location(:,:) ~= 0,2)~=0;
        ptCloud = pointCloud(ptCloud.Location(idx,:));
    
        %% pcd on img
        eulerAngle1 = lidar_angles;
        translation =  - camera_position + lidar_position;
        R1 = eul2rotm(eulerAngle1);
        A1 = [[R1;0,0,0],[translation';1]];
        
        % [depth,cameraPoints] = func_ptc_transformer(ptCloud,intrinsics,A1);
        [depth,~] = func_projectLidarToDepthImage(ptCloud, A1, focalLength, principalPoint, imageSize, RadialDistortion6, TangentialDistortion);
        % pause(5)
        %% show depth onto img
        hold on
        scatter(depth(:,1),depth(:,2),5,depth(:,3),'MarkerEdgeColor','flat','MarkerFaceColor','flat')
        colormap(turbo);
        clim([0 100])
        hold off
    end
    drawnow;
    frame = getframe(img_fig);
    writeVideo(video,frame);
end
close(video)