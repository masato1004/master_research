function unevenness_points = F_points_extractor(gradient_points,x_ref,y_ref,opt_params)
    
    road_width_from_center = interp1(x_ref,y_ref,gradient_points(:,1));
    gradient_idx = gradient_points(:,2) < road_width_from_center+opt_params.lane_width/2 & gradient_points(:,2) > road_width_from_center-opt_params.lane_width/2;% & gradient_points(:,1) < 14;

    road_gradient = pointCloud(gradient_points(gradient_idx,:));

    [~,~,outlierIndices] = pcfitplane(road_gradient,opt_params.plane_threshold,30000);
    unevenness_points = select(road_gradient,outlierIndices);
    % unevenness_points = road_gradient.Location(outlierIndices,:);
end