function [modified_data, plane, tform] = fitplane(data,threshold)
    %% PCFITPLANE
    max_distance = threshold;  % [m]
    model = pcfitplane(data,max_distance,MaxNumTrials=1500);
    new_ver = data.Location;
    X_max = max(new_ver(:,1),[],'all');
    X_min = min(new_ver(:,1),[],'all');
    Y_max = max(new_ver(:,2),[],'all');
    Y_min = min(new_ver(:,2),[],'all');
    [X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
    Z_pre = -model.Parameters(1)*X_pre/model.Parameters(3) - model.Parameters(2)*Y_pre/model.Parameters(3) - model.Parameters(4)/model.Parameters(3);

    %% ROTATION
    % Find a normal vector of RANSAC-surface
    [nx,ny,nz]=surfnorm(X_pre,Y_pre,Z_pre); % find normal vector
    s_vec=reshape([nx,ny,nz],9,9,3);        % combine normal vector
    u_vec = reshape(s_vec(5,5,:),1,3);          % pick-up one vector

    % Find a rotation matrix
    z_vec = [0,0,1];                        % z vector
    angle_r = subspace(z_vec',u_vec');          % [rad] angle
    r_axis = cross(z_vec,u_vec);                % find a axis of rotation

    % Rotate the point cloud
    plane = reshape([X_pre,Y_pre,Z_pre],9,9,3);
    plane = reshape(plane,81,3);
    plane = pointCloud(plane);

    rotationVector = angle_r * r_axis/norm(r_axis);
    rotationMatrix = rotvec2mat3d(rotationVector);
    translation = [0 0 0];
    tform = rigid3d(rotationMatrix,translation);
    ptOut = pctransform(data,tform);
    plane = pctransform(plane,tform);
    
    % Lift the road up for camera-height by mesh
    modified_data = ptOut.Location;
    modified_data(:,3) = modified_data(:,3) - mean(plane.Location(:,3));
    modified_data = pointCloud(modified_data);
end