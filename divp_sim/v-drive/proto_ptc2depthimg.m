close all;

func_ptc2depthimage()

function func_ptc2depthimage()
    close all;
    sim_time = 100;
    max_depth = 20;

    %% define pcd and img
    lidar_name = "roof"

    pcd_dir_name = "pcd_"+lidar_name+"-lidar_loop1";
    img_dir_name = "image_front_loop1";

    pcd_list = dir(pcd_dir_name+"/*.pcd");
    img_list = dir(img_dir_name+"/*.png");

    last_len = 0;

    save_dir = "sparse_depth_loop1";
    if ~exist(save_dir,'dir')
        mkdir(save_dir)
    end

    %% define ego csv and load positions
    % filename = "scenario/dataset_2/scenario_1_divp_Veh_NissanXtrail_1.csv";
    % opts = detectImportOptions(filename);
    % opts.SelectedVariableNames = ["timestamp", "pos_x","pos_y","pos_z", "yaw_rad", "pitch_rad", "roll_rad"];
    % pos_table = readtable(filename,opts);

    %% video setting
    sampling_freq = 40;
    sampling_period = 1/sampling_freq;

    %% camera parameter
    img_w = 3840;
    img_h = 2160;
    % k1 = 1.36648;
    % k2 = 1.79417;
    % k3 = 0.1704;
    % k4 = 1.90693;
    % k5 = 2.64875;
    % k6 = 0.97058;
    % p1 = 0.00014;
    % p2 = -0.00008;
    k1 = 0;
    k2 = 0;
    k3 = 0;
    k4 = 0;
    k5 = 0;
    k6 = 0;
    p1 = 0;
    p2 = 0;
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
    % camera_position = [1.690000, 0.0, 1.500000];
    camera_position = [1.881159, 0.0, 1.554000];
    camera_angles = [0., 0., 0.];

    
    while last_len < sim_time/0.025 + 1
        pcd_list = dir(pcd_dir_name+"/*.pcd");
        img_list = dir(img_dir_name+"/*.png");

        len_pcd_list = length(pcd_list);
        len_img_list = length(img_list);

        if min([len_pcd_list,len_img_list]) ~= last_len
            
            imax = min([len_pcd_list,len_img_list]);
            pause(3)
            for i = last_len+1:1:imax
                pcd_name = pcd_list(i).name;
                img_name = img_list(i).name;
                
                %% load img
                img = imread(img_dir_name+"/"+img_name);
                sparse_depth = uint16(zeros(height(img),width(img)));
                % uneven_label = uint16(zeros(height(img),width(img)));
                
                if pcd_name ~= "lidar1000.000.pcd" && pcd_name ~= "lidar1000000.pcd"
                    %% load pcd
                    ptCloud = pcread(pcd_dir_name+"/"+pcd_name);
                    idx = sum(ptCloud.Location(:,:) ~= 0,2)~=0;
                    ptCloud = pointCloud(ptCloud.Location(idx,:));
                
                    %% pcd on img
                    eulerAngle1 = lidar_angles;
                    translation =  - camera_position + lidar_position;
                    R1 = eul2rotm(eulerAngle1);
                    A1 = [[R1;0,0,0],[translation';1]];
                    ptCloud = pctransform(ptCloud,rigidtform3d(A1));

                    % road_points = pointCloud(ptCloud.Location(ptCloud.Location(:,3)<-0.5&ptCloud.Location(:,2)<2&ptCloud.Location(:,2)>-2&ptCloud.Location(:,1)>0,:));
                    % [~, plane_mesh, ~, outlier_idx] = func_fitPlane(road_points,road_points,0.005);
                    % disp(size(road_points.Location)+"/"+size(outlier_idx))
                    % label_pt = select(road_points,outlier_idx);
                    % pcshow(label_pt)
                    % drawnow;
                    
                    [depth,~] = func_projectLidarToDepthImage(ptCloud, eye(4), focalLength, principalPoint, imageSize, RadialDistortion6, TangentialDistortion);
                    % [label_depth,~] = func_projectLidarToDepthImage(label_pt, eye(4), focalLength, principalPoint, imageSize, RadialDistortion6, TangentialDistortion);

                    ind = sub2ind([height(img),width(img)],round(depth(:,2)),round(depth(:,1)));
                    sparse_depth(ind) = uint16(round(depth(:,3)*(65535/max_depth)));
                    % imshow(sparse_depth)
                    % drawnow;

                    % label_ind = sub2ind([height(img),width(img)],round(label_depth(:,2)),round(label_depth(:,1)));
                    % uneven_label(label_ind) = uint16(65535);
                    % end
                    imwrite(sparse_depth, save_dir+"/depth_"+img_name, "BitDepth",16);
                    % imwrite(uneven_label, save_label_dir+"/label_"+img_name, "BitDepth",16);
                    disp(i)
                end
            end
        end
        last_len = min([len_pcd_list,len_img_list]);
        disp(last_len+"/"+(sim_time/0.025 + 1));
        pause(5);
    end
end