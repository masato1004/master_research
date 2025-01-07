close all;

%% configuration
use_path = true;
use_ptc = true;
use_img = false;

used = [use_path,use_ptc,use_img];

animation = false;

%% define pcd and img

pcd_dir_name = "pcd_gtpoints";
img_dir_name = "image";

pcd_list = dir(pcd_dir_name+"/*.pcd");
img_list = dir(img_dir_name+"/*.png");

save_dir_name = "gtPcdSLAM";
if ~exist(save_dir_name,'dir')
    mkdir(save_dir_name)
end

%% define ego csv and load positions
filename = "scenario/manhole/scenario_1_divp_Veh_NissanXtrail_1.csv";
opts = detectImportOptions(filename);
opts.SelectedVariableNames = ["timestamp", "pos_x","pos_y","pos_z", "yaw_rad", "pitch_rad", "roll_rad"];
pos_table = readtable(filename,opts);

%% video setting
sampling_freq = 20;
sampling_period = 1/sampling_freq;

if animation
    datas = ["path","ptc","img"];
    used_data = datas(used);
    filename = "";
    for i = 1:length(used_data)
        filename=append(filename,used_data(i));
    end
    videoname = save_dir_name+"/"+filename+lidar_name;
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

% camera_position = [1.690000, 0.0, 1.500000];
camera_position = [1.881159, 0.0, 1.554000];
camera_angles = [0., 0., 0.];

%% figure
img_fig = figure('Position',[300,300,865.67,524.67]);


imax = (max(pos_table.timestamp)/sampling_period)+1;
strlen = 0;
total_pcd=[];
for i = 897:5:imax
    pcd_name = pcd_list(i).name;
    % img_name = img_list(i).name;
    
    %% load img
    % img = imread(img_dir_name+"/"+img_name);
    
    %% imshow
    % if use_img
    %     imshow(img);
    % end

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

    %% load pcd
    if i > 1
        ptCloud = pcread(pcd_dir_name+"/"+pcd_name);
        idx = ptCloud.Location(:,3)<20&ptCloud.Location(:,1)<20&ptCloud.Location(:,1)>0&ptCloud.Location(:,2)<5&ptCloud.Location(:,2)>-5;
        ptCloud = [ptCloud.Location(idx,1),-ptCloud.Location(idx,2),ptCloud.Location(idx,3)];
        bonnet_idx = ptCloud(:,1)<3&ptCloud(:,2)<2.5&ptCloud(:,2)>-2.5&ptCloud(:,3)>-1.2;
        ptCloud = pointCloud(ptCloud(~bonnet_idx,:));

        R1 = eul2rotm(agl);
        A1 = [[R1;0,0,0],[pos';1]];
        ptCloud = pctransform(pointCloud([ptCloud.Location(:,1)+camera_position(1),ptCloud.Location(:,2),ptCloud.Location(:,3)+camera_position(3)]),rigidtform3d(A1));

        gridStep = 0.015;
        ptCloud = pcdownsample(ptCloud,'gridAverage',gridStep);

        pcshow(ptCloud,"MarkerSize",2);pcshow(ptCloud); hold on;
        set(gcf,'color','w');
        set(gca,'color','w');
        set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
        xlabel( '\itX \rm[m]'); ylabel( '\itY \rm[m]'); zlabel( '\itZ \rm[m]');

        fontname(gcf,"Arial");
        fontsize(img_fig().Children,13,"points");
        % lidar3 = scatter3(ptCloud.Location(:,1),ptCloud.Location(:,2),ptCloud.Location(:,3),2,ptCloud.Location(:,3),'fill');  hold on;

        drawnow;

        total_pcd = [total_pcd; ptCloud.Location];
    end
    Tmp = {'Progress: %3d/%d\n', i, imax};
    Tmp{1} = [ repmat(sprintf('\b'),[1 strlen]),  Tmp{1} ];

    Txt = sprintf(Tmp{1:3});
    fprintf(Txt);
    strlen = length(Txt) - strlen;
end
pcwrite(pointCloud(total_pcd),save_dir_name+"/map_gt",'Encoding','ascii');