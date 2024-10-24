%% define data folders
% dataset="val_selection_cropped/";
% dataset="val_selection_cropped_fixed_supervision/";
% dataset="val_selection_cropped_labeled_supervision/";
dataset=uigetdir("./val_selection/", "DATASET folder to Open") + "\";

% results="unpretrained_unfixed_supervision/results/";
% results="unpretrained_fixed_supervision/results/";
% results="pretrained_fixed_supervision/results/";
% results="pretrained_labeled_supervision/results/";
% results="conventional_model/results/";
results=uigetdir("./results/","RESULTS folder to Open") + "\results\";

list_predicted_imgs = dir(results+"*.png");
list_rawlidar_imgs = dir(dataset+"velodyne_raw/*.png");
list_color_imgs = dir(dataset+"image/*.png");
groundtruth_imgs = dir(dataset+"groundtruth_depth/*.png");

%% read datas
close all;

file_name = "1708412721.142365.png";
file_num = 0;
flag = true;
while flag
    file_num = file_num+1;
    name = list_predicted_imgs(file_num).name;
    if name==file_name
        flag=false;
    end
    if file_num == length(list_predicted_imgs)-1
        flag=false;
    end
end
file_num=182;
% file_num=175;
% file_num=141;
% file_num=210;
% file_num=207;

rawlidarImage_read = imread(dataset+"velodyne_raw/"+list_rawlidar_imgs(file_num).name);
predictedImage_read = imread(results+list_predicted_imgs(file_num).name);
colorImage_read = imread(dataset+"image/"+list_color_imgs(file_num).name);
groundtruth_read = imread(dataset+"groundtruth_depth/"+groundtruth_imgs(file_num).name);
if exist(dataset+"uneven_label/","dir")
    labelImage_read = imread(dataset+"uneven_label/"+list_color_imgs(file_num).name);
end

imshow(labelImage_read)
% imshow(rawlidarImage_read)
% imshow(groundtruth_read)
colormap(turbo)
colormap(gray)
clim([0 65535]);
% colorbar;
fontname(gcf,"Arial");
fontsize(gca,8,"points");


% depthImage_read = predictedImage_read;

% depthImage_check  = double(depthImage_read);
% groundtruth_check = double(groundtruth_read);
% depthImage_check(depthImage_check==0)   = nan;
% groundtruth_check(groundtruth_check==0) = nan;
% rmse_px = rmse(depthImage_check.*15./65535,groundtruth_check.*15./65535,"omitnan");
% rmse_px = rmmissing(rmse_px);
% RMSE_on_depthmap = sum(rmse_px,'all')/numel(rmse_px)
% 
% %% create original size images for pcfromdepth as new ones
% 
% groundtruth_original_size=uint16(zeros(1080, 1920));
% depthImage_original_size=uint16(zeros(1080, 1920));
% raw_depth_original_size=uint16(zeros(1080, 1920));
% colorImage_original_size=uint8(ones(1080, 1920, 3));
% 
% start_x = 353-1;
% start_y = 449-1;
% rect_width = 1216-1;
% rect_height = 264-1;
% % start_x = 353-1;
% % start_y = 449-1;
% % rect_width = 1216-1;
% % rect_height = 352-1;
% % start_x = 503-1;
% % start_y = 449-1;
% % rect_width = 1216-1;
% % rect_height = 150-1;
% depthImage_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width) = depthImage_read;
% groundtruth_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width,:) = groundtruth_read;
% colorImage_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width,:) = colorImage_read;
% raw_depth_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width,:) = rawlidarImage_read;
% depthImage = depthImage_original_size;
% groundtruth = groundtruth_original_size;
% rawlidarImage = raw_depth_original_size;
% colorImage = colorImage_original_size;