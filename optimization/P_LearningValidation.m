%% Load python function
python_path = "C:\Users\"+getenv('username')+"\research\divpenv\Scripts\python.exe";
if pyenv().Executable ~= python_path
    pe = pyenv(Version=python_path);
end
pymod = py.importlib.import_module('F_depthcompletion');
py.importlib.reload(pymod);

%% Load data
data_path = "C:\Users\"+getenv('username')+"\research\master_research\divp_sim\v-drive\dataset\test_depth_completion_anonymous"; % val_selection_cropped
output_dir = "dense_map";
if ~exist(fullfile(data_path, output_dir), 'dir')
    mkdir(fullfile(data_path, output_dir));
end

% paths
rgb_dir = dir(fullfile(data_path, 'image', '*.png'));
depth_dir = dir(fullfile(data_path, 'velodyne_raw', '*.png'));
label_dir = dir(fullfile(data_path, 'uneven_label', '*.png'));
gt_dir = dir(fullfile(data_path, 'groundtruth_depth', '*.png'));

crop_h = 592;
crop_w = 1512;
sf = 20/65535;

rmse_all_list = zeros(1,length(rgb_dir));
mae_all_list = zeros(1,length(rgb_dir));
rmse_label_list = [];
mae_label_list = [];
calc_time_list = zeros(1,length(rgb_dir));

strlen = 0;
imax = length(rgb_dir);
for i = 1:imax
    % Load data
    rgb = imread(fullfile(rgb_dir(i).folder, rgb_dir(i).name));
    depth = imread(fullfile(depth_dir(i).folder, depth_dir(i).name));
    label = imread(fullfile(label_dir(i).folder, label_dir(i).name)); label(label>0) = (label(label>0))./max((label(:))); label = logical(label);
    gt = imread(fullfile(gt_dir(i).folder, gt_dir(i).name));
    
    colorImage_np = py.numpy.array(rgb);
    rawlidarImage_np = py.numpy.array(depth,dtype=py.numpy.uint16);
    
    tic;
    output = py.F_depthcompletion.depth_completion(colorImage_np, rawlidarImage_np, crop_h, crop_w);
    calc_time = toc;
    calc_time_list(i) = calc_time;
    dense_map = reshape(uint16(output),[crop_h,crop_w]);

    rmse_all = rmse(double(gt), double(dense_map),'all')*sf;
    mae_all = mean(abs(gt-dense_map),'all')*sf;

    rmse_all_list(i) = rmse_all;
    mae_all_list(i) = mae_all;

    if sum(label(:)) ~=0
        sum(label(:))
        rmse_label = rmse(double(gt(label)), double(dense_map(label)),'all')*sf
        mae_label = mean(abs(gt(label)-dense_map(label)),'all')*sf
        
        rmse_label_list = [rmse_label_list, rmse_label];
        mae_label_list = [mae_label_list, mae_label];
    end

    % Save data
    imwrite(dense_map, fullfile(data_path, output_dir, depth_dir(i).name));

    Tmp = {'Progress: %3d/%d\n', i, imax};
    Tmp{1} = [ repmat(sprintf('\b'),[1 strlen]),  Tmp{1} ];

    Txt = sprintf(Tmp{1:3});
    fprintf(Txt);
    strlen = length(Txt) - strlen;
end
rmse_all_mean = mean(rmse_all_list)
mae_all_mean = mean(mae_all_list)
rmse_label_mean = mean(rmse_label_list)
mae_label_mean = mean(mae_label_list)
calc_time_mean = mean(calc_time_list)