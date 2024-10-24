% LOADING
prev_start = 3;
prev_end = 5.5;
prev_time = 1708412721.142365;  % file name of when 3m far from the bump
current_dis = r_p_prev(1,1);
if sensing
    load_dir = "../preview_datas";
    listing = dir(load_dir+"/*.mat");
    file = load(load_dir + "/" + listing(2).name);
    vertices = file.vertices;
    
    wf_local = previewing(vertices);
    data_end = wf_local(2,round(end/2));
    wf_global = wf_local; last_minimum = 1.9;
else
    if realworld
        % learning_condition = "pretrained_fixed_supervision";
        root = "C:/Users/INOUE MASATO/research/master_research/sensing/rosbag_reader/ouster-dual/";
        est_dir = root+"results/"+learning_condition+"/results/";
        raw_dir = root+"val_selection/val_selection_cropped/velodyne_raw/";
        list_estimated_imgs = dir(est_dir + "*.png");
        list_rawlidar_imgs = dir(raw_dir + "*.png");

        listing = list_estimated_imgs;

        file_name = "1708412721.142365.png";
        file_num = 0;
        found = true;
        while found
            file_num = file_num+1;
            name = listing(file_num).name;
            if name==file_name
                found=false;
            end
            if file_num == length(listing)-1
                found=false;
            end
        end
        depthImage_read = imread(fullfile(listing(file_num).folder,listing(file_num).name));
        
        addpath("../")
        wf_local = func__ptc2profile(func__depth2ptc(depthImage_read));
        data_end = wf_local(2,round(end));
    else
        listing = [];
        wf_local = [
            prev_start:tc*V:prev_end;
            makima(r_p_prev(1,:),r_p_prev(2,:),current_dis+prev_start:tc*V:current_dis+prev_end)
            ];
    end
    wf_global = wf_local; last_minimum = wf_global(1,1);
end
wf_grad = [
    wf_global(1,:)./V;
    wf_global(2,:);
    gradient(wf_global(2,:))./(gradient(wf_global(1,:))./V)
    ];
dw_prev = zeros(4,M+1);