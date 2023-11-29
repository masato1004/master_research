function prev_profile = previewing(vertices)
    %% lidar
    % ptCloud = pcread("C:\Users\masato\research\master_research\sensing\ouster-lidar\OusterLiDARply\stay\2023-11-28-17-23-33\100000132-17-23-40-61165.ply");
    %% stereo camera
    ptCloud = pcread("C:\Users\masato\research\master_research\sensing\zed-matlab\matlab\ZED2iply\-stay-\2023-11-28-17-23-32\100000085-17-23-41-61170.ply");
    vertices = ptCloud.Location;
    %% ROTATE POINT CLOUD TO MATCH THE ZED-COORDINATE
    x_vec = [1,0,0];
    y_vec = [0,1,0];
    z_vec = [0,0,1];
    rotationVector = (pi/2) * z_vec/norm(z_vec);
    rotationMatrix = rotationVectorToMatrix(rotationVector);
    translation = [0 0 0];
    tform = rigid3d(rotationMatrix,translation);
    ptCloudOut = pctransform(ptCloud,tform);
    vertices = ptCloudOut.Location;
    % figure
    % pcshow(ptCloudOut)
    
    % LOAD POINT CLOUD
    
    %% PICK UP ROAD POINT CLOUD
    road_data = vertices(vertices(:,1)>=-1.2 & vertices(:,1)<=1.2,:,:);
    road_data = pointCloud(road_data);
    ptCloudA = road_data;
    % figure;
    % pcshow(raod_cloud);
    
    % DOWN SAMPLING
    % gridStep = 0.05;
    % ptCloudA = pcdownsample(road_data,'gridAverage',gridStep);
    % figure;
    % pcshow(ptCloudA);
    
    %% RANSAC
%     RANSAC_K = 700;
%     RANSAC_T = 0.02;
%     RANSAC_R = 0.4;
%     new_ver = ptCloudA.Location;
%     data_size = size(new_ver);
%     data_num = data_size(1);
%     [a_,b_,c_,d_] = ransac_func(new_ver, RANSAC_K, RANSAC_T, fix(data_num*RANSAC_R));
%     X_max = max(new_ver(:,1),[],'all');
%     X_min = min(new_ver(:,1),[],'all');
%     Y_max = max(new_ver(:,2),[],'all');
%     Y_min = min(new_ver(:,2),[],'all');
%     [X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
%     Z_pre = -a_*X_pre/c_ - b_*Y_pre/c_ - d_/c_;
%     hold on;
%     % mesh2 = mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","g","FaceColor","none");
    
    %% PCFITPLANE
    max_distance = 0.02;  % [m]
    model = pcfitplane(ptCloudA,max_distance);
    new_ver = ptCloudA.Location;
    X_max = max(new_ver(:,1),[],'all');
    X_min = min(new_ver(:,1),[],'all');
    Y_max = max(new_ver(:,2),[],'all');
    Y_min = min(new_ver(:,2),[],'all');
    [X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
    Z_pre = -model.Parameters(1)*X_pre/model.Parameters(3) - model.Parameters(2)*Y_pre/model.Parameters(3) - model.Parameters(4)/model.Parameters(3);
%     hold on;
%     mesh2 = mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","r","FaceColor","none");

    %% ROTATION
    % Find a normal vector of RANSAC-surface
    % zlim([-2,6])
    % ylim([-1,7])
    % xlim([-2,6])
    [nx,ny,nz]=surfnorm(X_pre,Y_pre,Z_pre); % find normal vector
    s_vec=reshape([nx,ny,nz],9,9,3);        % combine normal vector
    u_vec = reshape(s_vec(5,5,:),1,3);          % pick-up one vector
    
    % Find a rotation matrix
    z_vec = [0,0,1];                        % z vector
    angle_r = subspace(z_vec',u_vec');          % [rad] angle
    r_axis = cross(z_vec,u_vec);                % find a axis of rotation
    
    % Rotate the point cloud
    new_mesh = reshape([X_pre,Y_pre,Z_pre],9,9,3);
    new_mesh = reshape(new_mesh,81,3);
    new_mesh = pointCloud(new_mesh);
    
    % stereo camera angle -> -25.93
    rotationVector = -angle_r * r_axis/norm(r_axis);
    rotationMatrix = rotationVectorToMatrix(rotationVector);
    translation = [0 0 0];
    tform = rigid3d(rotationMatrix,translation);
    ptCloudOut = pctransform(road_data,tform);
    new_mesh_out = pctransform(new_mesh,tform);
    
    % Lift the road up for camera-height by mesh
    ptc_datas = ptCloudOut.Location;
    ptc_datas(:,3) = ptc_datas(:,3) - mean(new_mesh_out.Location(:,3));
    ptc_datas = ptc_datas(ptc_datas(:,3)<=0.3,:,:);
    % figure;
    % pcshow(pointCloud(xyz));
    
    %% PICK UP AS 2D
    range_min = 5.06;     % minimum measurable distance [m]
    range_max = 7;        % maximum measurable distance [m]
    pick_up_width = 0.24; % width of datas for a road profile [m]
    pick_up_center = 0;   % center of pick up position [m]
    
    p_min = pick_up_center - pick_up_width/2;
    p_max = pick_up_center + pick_up_width/2;
    
    line = ptc_datas(ptc_datas(:,1)>=p_min & ptc_datas(:,1)<=p_max & ptc_datas(:,2)<=range_max & ptc_datas(:,2)>=range_min,:,:);
    % line = ptc_datas(ptc_datas(:,1)>=-0.075 & ptc_datas(:,1)<=0.075 & ptc_datas(:,2)<=7 & ptc_datas(:,2)>=5.06,:,:);
    [~,ind] = sort(line(:,2));
    prev_profile=line(ind,2:3)';
    
    % fig = figure('name', "Road Displacement from ZED",'Position', [500-20 500-20 1400 380]);
    % plot(line_neo(:,2),line_neo(:,3),"LineWidth",1,"Color","#0000ff");
    
end

%% RANSAC func
function [a,b,c,d] = ransac_func(data,k,t,r)
arguments
    data = [0 0 0];
    k = 100; % max loop num
    t = 0.075; % threshold error val for inlier
    r = 800; % requrired inlier sample num to be correnct param
end
    good_models = [];
    good_model_errors = [];
    iterations = 0;
    while iterations < k
        % random sampling
        [len, ~] = size(data);
        sample = data(randsample(len,3), :);
        [a_,b_,c_,d_] = getParamWithSamples(sample);
        param = [a_,b_,c_,d_];
        inliers = [];
        for s = 1:len
            p = data(s,:);
            % checking not to sample the other samples or through
            if ~logical(sum(p == sample))
                if getError(param, p) > t
                    continue
                else
                    inliers = [inliers; p];
                end
            end
        end
        % add model to good_model if the it fit well enough
        if length(inliers) > r
            errorlist = [];
            for s = 1:len
                p = data(s,:);
                errorlist = [errorlist; getError(param, p)];
                current_error = mean(errorlist, "all");
            end
            good_models = [good_models; param];
            good_model_errors = [good_model_errors; current_error];
        end
        iterations = iterations + 1;
    end
        
    [~, best_index] = min(good_model_errors);
    a = good_models(best_index,1);
    b = good_models(best_index,2);
    c = good_models(best_index,3);
    d = good_models(best_index,4);
end

%% detect params from 3 points
% sample: 3-3 matrix
function [a,b,c,d] =  getParamWithSamples(samples)
    p1 = samples(1,:);
    p2 = samples(2,:);
    p3 = samples(3,:);
    x1 = p1(1);
    x2 = p2(1);
    x3 = p3(1);
    y1 = p1(2);
    y2 = p2(2);
    y3 = p3(2);
    z1 = p1(3);
    z2 = p2(3);
    z3 = p3(3);
    a = (y1*z2 - y1*z3 - y2*z1 + y2*z3 + y3*z1 - y3*z2);
    b = (-x1*z2 + x1*z3 + x2*z1 - x2*z3 - x3*z1 + x3*z2);
    c = (x1*y2 - x1*y3 - x2*y1 + x2*y3 + x3*y1 - x3*y2);
    d = (-x1*y2*z3 + x1*y3*z2 + x2*y1*z3 - x2*y3*z1 - x3*y1*z2 + x2*y2*z1);
end

%% get error-average between pre and cor
function error = getError(model, p)
    x = p(1);
    y = p(2);
    z = p(3);
    error = abs(applyModel(model, x, y) - z);
end

%% calculate z with predicted model
function z = applyModel(model, x, y)
    a = model(1);
    b = model(2);
    c = model(3);
    d = model(4);
    z = -a*x/c - b*y/c - d/c;
end