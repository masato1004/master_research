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

list_estimated_imgs = dir(results+"*.png");
list_rawlidar_imgs = dir(dataset+"velodyne_raw/*.png");
list_color_imgs = dir(dataset+"image/*.png");
groundtruth_imgs = dir(dataset+"groundtruth_depth/*.png");

%% read datas
close all;

file_name = "1708412721.496111.png";
file_num = 0;
flag = true;
while flag
    file_num = file_num+1;
    name = list_estimated_imgs(file_num).name;
    if name==file_name
        flag=false;
    end
    if file_num == length(list_estimated_imgs)-1
        flag=false;
    end
end
file_num=182;
% file_num=175;
% file_num=141;
% file_num=210;
% file_num=207;

temp_fig = figure("Position",[100,100,1000,400]);
mean_data_num = [50, 50]; % 1200 1200
range_min = 0;        % minimum measurable distance [m]
range_max = 7;        % maximum measurable distance [m]

% for file_num = 170:length(list_estimated_imgs)
animation = true;

if animation
    video = VideoWriter("sparse_roaddem_from_32",'MPEG-4');
    video.FrameRate = 2;
    open(video);
end
for file_num = 160:250
    rawlidarImage_read = imread(dataset+"velodyne_raw/"+list_rawlidar_imgs(file_num).name);
    estimatedImage_read = imread(results+list_estimated_imgs(file_num).name);
    colorImage_read = imread(dataset+"image/"+list_color_imgs(file_num).name);
    groundtruth_read = imread(dataset+"groundtruth_depth/"+groundtruth_imgs(file_num).name);

    chosen_depth = rawlidarImage_read;

    if sum(sum(rawlidarImage_read~=0)) < 2400
        disp(list_estimated_imgs(file_num).name)
        disp(file_num)
        upsampled_points = func__depth2ptc(chosen_depth,colorImage_read);
        upsampled_points_eliminate_idx = upsampled_points.Location(:,1)>3&upsampled_points.Location(:,1)<5.5&upsampled_points.Location(:,3)>-0.03;
        upsampled_points = pointCloud(upsampled_points.Location(upsampled_points_eliminate_idx,:,:),Color=upsampled_points.Color(upsampled_points_eliminate_idx,:,:));
        f_prev_profile = func__ptc2profile(upsampled_points);

        % point cloud
        % pcshow(upsampled_points);
        pcshow(reshape(upsampled_points.Location,[],3));
        xlabel("\itX \rm[m]");
        ylabel("\itY \rm[m]");
        zlabel("\itZ \rm[m]");
        fontname(gcf,"Times New Roman");
        fontsize(gca,20,"points");
        set(gcf,'color','w');
        set(gca,'color','w');
        set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
        
        % profile
        % raw_profile = scatter(f_prev_profile(1,:),f_prev_profile(2,:),1.5,'filled',"MarkerFaceColor","#0000ff","DisplayName","Raw 64 channel Data"); hold on;  % picked up points
        % profile = plot(f_prev_profile(1,:),movmean(f_prev_profile(2,:),mean_data_num),"LineWidth",5,"LineStyle",":","Color","#ff0000","DisplayName","Moving Average");
        % xlim([range_min,range_max]);
        % ylim([-0.08, 0.08]);
        % % axis equal;
        % xlabel("Local Distance from Front Wheel [m]");
        % ylabel("Previewed Displacement [m]");
        % % title("Front LiDAR");
        % legend("Location","southwest");
        % % interpolate
        % % f_poly = polyfit(f_prev_profile(1,:),f_prev_profile(2,:),5);
        % % f_interplated = f_poly(1)*f_prev_profile(1,:).^5 + f_poly(2)*f_prev_profile(1,:).^4 + f_poly(3)*f_prev_profile(1,:).^3 + f_poly(4)*f_prev_profile(1,:).^2 + f_poly(5)*f_prev_profile(1,:) + f_poly(6);
        % % plot(f_prev_profile(1,:),f_interplated,"LineWidth",2,"Color","#ff0000","LineStyle","--");
        % xlim([range_min,range_max]);
        % ylim([-0.075,0.075]);
        % xlabel("\itX \rm[m]");
        % ylabel("\itZ \rm[m]");
        % grid on
        % fontname(gcf,"Times New Roman");
        % fontsize(gca,20,"points");
        % grid on;

        drawnow
        if animation
            frame = getframe(temp_fig);
            writeVideo(video,frame);
        end

        % delete(raw_profile)
        % delete(profile)
    end
end
if animation
    close(video)
end