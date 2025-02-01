function cropped_img = F_datacropper(clock, name, img, start_y, start_x, rect_height, rect_width)
    % F_datacropper: function to crop the image
    % Input:
    %   img: image
    %   crop_h: height of the cropped image
    %   crop_w: width of the cropped image
    % Output:
    %   cropped_img: cropped image

    save_name = name+ num2str(clock+1000,'%.3f') + ".png";

    persistent save_dir
    if isempty(save_dir)
        save_dir = name;
        if ~exist(save_dir,'dir')
            mkdir(save_dir)
        end
    end

    cropped_img = img(start_y:start_y+rect_height ,start_x:start_x+rect_width, :);
end