function sparse_depth = func_ptc2depthimg(clock, points, img)
    persistent max_depth cam_params lidar_position lidar_angles camera_position save_dir img_name
    if isempty(max_depth)
        max_depth = 20;

        %% camera parameter
        cam_params = struct();
        cam_params.img_w = 3840;
        cam_params.img_h = 2160;
        cam_params.k1 = 1.36648;
        cam_params.k2 = 1.79417;
        cam_params.k3 = 0.1704;
        cam_params.k4 = 1.90693;
        cam_params.k5 = 2.64875;
        cam_params.k6 = 0.97058;
        cam_params.p1 = 0.00014;
        cam_params.p2 = -0.00008;
        cam_params.fx = 2445.66438;
        cam_params.fy = 2444.75377;
        cam_params.cx = 1905.44853;
        cam_params.cy = 1073.60153;
        imageSize = [cam_params.img_h, cam_params.img_w];
        focalLength      = [cam_params.fx, cam_params.fy];
        principalPoint   = [cam_params.cx, cam_params.cy];
        RadialDistortion6 = [cam_params.k1, cam_params.k2, cam_params.k3, cam_params.k4, cam_params.k5, cam_params.k6];
        TangentialDistortion = [cam_params.p1,cam_params.p2];
        % intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",RadialDistortion,"TangentialDistortion",TangentialDistortion);

        %% cam2roof-lidar transformation
        lidar_position = [1.110943, 0., 1.998877];
        lidar_angles = [0., 0., 0.];
        camera_position = [1.881159, 0.0, 1.554000];

        save_dir = "sparse_depth";
        if ~exist(save_dir,'dir')
            mkdir(save_dir)
        end

        img_name = num2str(clock+1000,'%.3f') + ".png";
    end
                
    %% load img
    sparse_depth = uint16(zeros(height(img),width(img)));
    
    %% load pcd
    points = points';
    idx = sum(points(:,:) ~= 0,2)~=0;
    ptCloud = pointCloud(points(idx,:));

    %% pcd on img
    eulerAngle1 = lidar_angles;
    translation =  - camera_position + lidar_position;
    R1 = eul2rotm(eulerAngle1);
    A1 = [[R1;0,0,0],[translation';1]];
    ptCloud = pctransform(ptCloud,rigidtform3d(A1));
    
    [depth,~] = func_projectLidarToDepthImage(ptCloud, eye(4), focalLength, principalPoint, imageSize, RadialDistortion6, TangentialDistortion);

    ind = sub2ind([height(img),width(img)],round(depth(:,2)),round(depth(:,1)));
    sparse_depth(ind) = uint16(round(depth(:,3)*(65535/max_depth)));

    imwrite(sparse_depth, save_dir+"/depth_"+img_name, "BitDepth",16);
end