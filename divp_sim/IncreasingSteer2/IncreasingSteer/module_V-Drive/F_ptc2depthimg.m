function sparse_depth = F_ptc2depthimg(clock, points, img, cam_params, dc_params)
    img_name = num2str(clock+1000,'%.3f') + ".png";
    persistent lidar_position lidar_angles camera_position save_dir
    if isempty(lidar_position)
        % intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",RadialDistortion,"TangentialDistortion",TangentialDistortion);

        %% cam2roof-lidar transformation
        lidar_position = [1.110943, 0., 1.998877];
        lidar_angles = [0., 0., 0.];
        camera_position = [1.881159, 0.0, 1.554000];

        save_dir = "sparse_depth";
        if ~exist(save_dir,'dir')
            mkdir(save_dir)
        end

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
    
    [depth,~] = func_projectLidarToDepthImage(ptCloud, eye(4), cam_params.focalLength, cam_params.principalPoint, cam_params.imageSize, cam_params.RadialDistortion6, cam_params.TangentialDistortion);

    ind = sub2ind([height(img),width(img)],round(depth(:,2)),round(depth(:,1)));
    sparse_depth(ind) = uint16(round(depth(:,3)*(65535/dc_params.maxCameraDepth)));

    imwrite(sparse_depth, save_dir+"/depth_"+img_name, "BitDepth",16);
end