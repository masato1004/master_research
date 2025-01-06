function func_getGtDepth(clock,depth,scale_factor,save_dir)

    depth_img_name = "depth"+ num2str(clock+1000,'%.3f') + ".png";

    depth(depth>scale_factor) = scale_factor;
    depth=uint16(round(depth.*(65535/scale_factor)));

    % persistent save_dir
    % if isempty(save_dir)
        % save_dir = "gt_depth";
        % listing = dir(save_dir+"/*.png");
        
    if ~exist(save_dir,'dir')
        mkdir(save_dir)
    end
    % end
    imwrite(depth, save_dir+"/"+depth_img_name, "BitDepth",16);
end