function [depth_info_on_img,cameraPoints] =  func_ptc_transformer(pcd,camIntrinsics,tmat_cam2lidar)
    % transform into image coordinate
    eulerAngle2 = [0 -pi/2 pi/2];
    R2 = eul2rotm(eulerAngle2);
    A2 = [[R2;0,0,0],[0;0;0;1]];
    
    A3 = A2*tmat_cam2lidar;

    % tform_campcd = rigidtform3d(tmat_cam2lidar);
    % campcd = pctransform(pcd,tform_campcd);
    
    tform = rigidtform3d(A);
    % [imPts,indices] = projectLidarPointsOnImage(campcd,camIntrinsics,tform);
    cameraPoints = pctransform(pcd,tform);

    % depth = sqrt(campcd.Location(indices,1).^2+campcd.Location(indices,2).^2+campcd.Location(indices,3).^2);
    depth = cameraPoints.Location(indices,3);
    depth_info_on_img = [imPts,depth];

end