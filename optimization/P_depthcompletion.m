%% define data folders
dataset=uigetdir("./", "DATASET folder to Open") + "\";

% results=uigetdir("../sensing/rosbag_reader/ouster-dual/results/","RESULTS folder to Open") + "\results\";

% list_predicted_imgs = dir(results+"*.png");
list_rawlidar_imgs  = dir(dataset+"velodyne_raw/*.png");
list_color_imgs     = dir(dataset+"image/*.png");
groundtruth_imgs    = dir(dataset+"groundtruth_depth/*.png");

%% Load python function
python_path = "C:\Users\"+getenv('username')+"\research\divpenv\Scripts\python.exe";
if pyenv().Executable ~= python_path
    pe = pyenv(Version=python_path);
end
pymod = py.importlib.import_module('F_depthcompletion');
py.importlib.reload(pymod);

%% read datas
close all;

% file_name = "depth_image_008850.png";
% file_num = 0;
% flag = true;
% while flag
%     file_num = file_num+1;
%     name = list_predicted_imgs(file_num).name;
%     if name==file_name
%         flag=false;
%     end
%     if file_num == length(list_predicted_imgs)-1
%         flag=false;
%     end
% end
file_num=40;

        
crop_h = 552;
crop_w = 1496;

disp(file_num)
rawlidarImage_read  = imread(dataset+"velodyne_raw/"+list_rawlidar_imgs(file_num).name);
% predictedImage_read = imread(results+list_predicted_imgs(file_num).name);
colorImage_read     = imread(dataset+"image/"+list_color_imgs(file_num).name);
groundtruth_read    = imread(dataset+"groundtruth_depth/"+groundtruth_imgs(file_num).name);

colorImage_np = py.numpy.array(colorImage_read);
rawlidarImage_np = py.numpy.array(rawlidarImage_read,dtype=py.numpy.uint16);
tic
output = py.F_depthcompletion.depth_completion(colorImage_np, rawlidarImage_np, crop_h, crop_w);
dense_map = reshape(uint16(output),[crop_h,crop_w]);
toc
imshow(dense_map)