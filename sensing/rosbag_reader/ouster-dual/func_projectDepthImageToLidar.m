function [pcd] = func_projectDepthImageToLidar(depthImage, fl, pp, rd, td)

    % Get image dimensions
    [height, width] = size(depthImage);
    
    % Create a grid of pixel coordinates
    [x, y] = meshgrid(1:width, 1:height);
    
    % Normalize pixel coordinates to the camera's principal point
    x_norm = (x - pp(1)) / fl(1);
    y_norm = (y - pp(2)) / fl(2);
    
    % Compute radial and tangential distortions
    r2 = x_norm.^2 + y_norm.^2; % Squared radius
    radial_distortion = (1+rd(1)*r2+rd(2)*r2.^2+rd(3)*r2.^3) ./ (1+rd(4)*r2+rd(5)*r2.^2+rd(6)*r2.^3);
    x_tangential = 2 * td(1) * x_norm .* y_norm + td(2) * (r2 + 2 * x_norm.^2);
    y_tangential = td(1) * (r2 + 2 * y_norm.^2) + 2 * td(2) * x_norm .* y_norm;
    
    % Apply distortion correction
    x_undistorted = x_norm .* radial_distortion + x_tangential;
    y_undistorted = y_norm .* radial_distortion + y_tangential;
    
    % Scale back to pixel coordinates
    % x_undistorted = x_undistorted * fl(1) + pp(1);
    % y_undistorted = y_undistorted * fl(2) + pp(2);
    
    % Depth values (Z-coordinate in meters)
    z = depthImage; % Depth is the Z-coordinate
    
    % Compute undistorted 3D coordinates
    x3D = x_undistorted.* z; % X-coordinate
    y3D = y_undistorted.* z; % Y-coordinate
    
    % Combine into a point cloud
    pointCloud = [x3D(:), y3D(:), z(:)];
    
    % Remove points with invalid depth (e.g., zero or NaN values)
    validPoints = z(:) > 0; % Adjust threshold as needed
    pcd = pointCloud(validPoints, :);

    % eulerAngle2 = [0 -pi/2 pi/2];
    % R2 = eul2rotm(-eulerAngle2);
    % A2 = [[R2;0,0,0],[0;0;0;1]];
    % tform = rigidtform3d(A2);
    % pcd = pctransform(pcd,tform);
end