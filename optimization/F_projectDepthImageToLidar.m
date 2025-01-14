function [pcd,validPoints,diffcolor,dpcd] = F_projectDepthImageToLidar(depthImage, fl, pp, rd, td, sf, rgbImg, crop_info)

    % Get image dimensions
    [img_H, img_W] = size(depthImage);
    
    % Create a grid of pixel coordinates
    [x, y] = meshgrid(1:img_W, 1:img_H);
    
    % Normalize pixel coordinates to the camera's principal point
    x_norm = (x - pp(1)) / fl(1);
    y_norm = (y - pp(2)) / fl(2);
    
    % Compute radial and tangential distortions
    r2 = x_norm.^2 + y_norm.^2; % Squared radius
    radial_distortion = (1+rd(1)*r2+rd(2)*r2.^2+rd(3)*r2.^3) ./ (1+rd(4)*r2+rd(5)*r2.^2+rd(6)*r2.^3);
    % radial_distortion = 1;
    x_tangential = 2 * td(1) * x_norm .* y_norm + td(2) * (r2 + 2 * x_norm.^2);
    y_tangential = td(1) * (r2 + 2 * y_norm.^2) + 2 * td(2) * x_norm .* y_norm;
    
    % Apply distortion correction
    x_undistorted = x_norm ;%./ radial_distortion - x_tangential;
    y_undistorted = y_norm ;%./ radial_distortion - y_tangential;
    
    % Scale back to pixel coordinates
    % x_undistorted = x_undistorted * fl(1) + pp(1);
    % y_undistorted = y_undistorted * fl(2) + pp(2);
    
    % Depth values (Z-coordinate in meters)
    z = double(depthImage)/sf; % Depth is the Z-coordinate
    
    % Compute undistorted 3D coordinates
    x3D = x_undistorted.* z; % X-coordinate
    y3D = y_undistorted.* z; % Y-coordinate
    
    % Combine into a point cloud
    [diffImage_x,diffImage_y] = gradient(double(y3D));
    x3D = x3D(crop_info{1},crop_info{2});
    y3D = y3D(crop_info{1},crop_info{2});
    z = z(crop_info{1},crop_info{2});
    rgbImg = rgbImg(crop_info{1},crop_info{2},:);

    % data_width = width(x3D);
    % data_height = height(x3D);
    % extract_height_idx = 1:2:data_height-20;
    % extract_width_idx = 1:3:data_width;
    % 
    % x3D = x3D(extract_height_idx,extract_width_idx);
    % y3D = y3D(extract_height_idx,extract_width_idx);
    % z = z(extract_height_idx,extract_width_idx);
    % rgbImg = rgbImg(extract_height_idx,extract_width_idx,:);


    pcd = [x3D(:), y3D(:), z(:)];
    color = double(reshape(rgbImg,[height(rgbImg)*width(rgbImg),3]))./255;
    % [pcd,indeices] = pcdownsample(pointCloud(pcd),'random',0.8);
    % [row,col,c] = ind2sub(size(y3D),indeices);

    diffImage = 1000000*(diffImage_x./sf).^2 + (diffImage_y./sf).^2;
    diffImage = diffImage(crop_info{1},crop_info{2});
    % diffImage(:,1)=0;
    % diffImage(1,:)=0;
    % diffImage(:,end)=0;
    % diffImage(end,:)=0;
    diffImage = diffImage./(z+x3D);
    diffcolor = double(reshape(diffImage,[height(diffImage)*width(diffImage),1]));
    
    % Remove points with invalid depth (e.g., zero or NaN values)
    validPoints = z(:) > 0; % Adjust threshold as needed
    pcd = pointCloud(pcd(validPoints,:),Color=color(validPoints,:));
    diffcolor=diffcolor(validPoints)*10000;
    % diffcolor(diffcolor>3.4) = 0;

    dpcd = pointCloud([pcd.Location(:,1),diffcolor+mean(pcd.Location(:,2)),pcd.Location(:,3)]);
    dpcd = pcdownsample(dpcd,'gridAverage',0.02);
    percentage = 0.5;
    % dpcd = pcdownsample(dpcd,'random',percentage,PreserveStructure=false);
    % eulerAngle2 = [0 -pi/2 pi/2];
    % R2 = eul2rotm(-eulerAngle2);
    % A2 = [[R2;0,0,0],[0;0;0;1]];
    % tform = rigidtform3d(A2);
    % pcd = pctransform(pcd,tform);
end