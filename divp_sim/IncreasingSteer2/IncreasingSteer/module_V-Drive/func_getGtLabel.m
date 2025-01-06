function func_getGtLabel(clock,label,save_dir)

    label_img_name = "label"+ num2str(clock+1000,'%.3f') + ".png";

    % label(label>scale_factor) = scale_factor;
    % label=uint16(round(label.*(65535/scale_factor)));

    label(label==1) = 0;
    label = label.*65535;
    label(label>65535) = 65535;
    label=uint16(label);

    % persistent save_dir
    % if isempty(save_dir)
        % save_dir = "gt_label";
        % listing = dir(save_dir+"/*.png");
        
    if ~exist(save_dir,'dir')
        mkdir(save_dir)
    end
    % end
    imwrite(label, save_dir+"/"+label_img_name, "BitDepth",16);
end