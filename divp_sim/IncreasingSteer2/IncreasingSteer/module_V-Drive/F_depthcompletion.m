function dense_depth = F_depthcompletion(clock, colorImage, rawlidarImage, crop_h, crop_w)

    % F_depthcompletion: function to complete the depth image using the color image and the raw lidar image
    % Input:
    %   colorImage: color image
    %   rawlidarImage: raw lidar image
    %   crop_h: height of the cropped image
    %   crop_w: width of the cropped image
    % Output:
    %   dense_depth: completed depth image

    persistent save_dir
    if isempty(save_dir)
        save_dir = "dense_depth";
        if ~exist(save_dir,'dir')
            mkdir(save_dir)
        end
    end

    depth_img_name = "dense_depth"+ num2str(clock+1000,'%.3f') + ".png";

    colorImage_np = py.numpy.array(colorImage);
    rawlidarImage_np = py.numpy.array(rawlidarImage,dtype=py.numpy.uint16);
    output = py.F_depthcompletion.depth_completion(colorImage_np, rawlidarImage_np, crop_h, crop_w);
    dense_depth = reshape(uint16(output),[crop_h,crop_w]);
    imwrite(dense_depth, save_dir+"/"+depth_img_name, "BitDepth",16);
end