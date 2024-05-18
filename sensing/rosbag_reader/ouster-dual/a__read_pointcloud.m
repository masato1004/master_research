warning('off','all');
%% load tf from calibration file
f_tf = readmatrix("calibration_file_zed_nissan_front.csv");
r_tf = readmatrix("calibration_file_zed_nissan_roof.csv");
f_rotm = quat2rotm([f_tf(7),f_tf(4:6)]);
r_rotm = quat2rotm([r_tf(7),r_tf(4:6)]);
f_translation = [f_tf(1),f_tf(2),f_tf(3)];
r_translation = [r_tf(1),r_tf(2),r_tf(3)];
f_tform = rigidtform3d(f_rotm,f_translation);
r_tform = rigidtform3d(r_rotm,r_translation);

% translat with position parameter from cad
rotate_angle_cam2wheel = [0 0 0];
f_translation_cam2wheel = [0 1 0]; % from cad
r_translation_cam2wheel = [-1 0 0]; % from cad
f_tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,f_translation_cam2wheel);
r_tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,r_translation_cam2wheel);

% calculate tfrom
f_tform.A = f_tform.A * f_tform_cam2wheel.A;
r_tform.A = r_tform.A * r_tform_cam2wheel.A;

%% lidar inclination
zed_inc_z = 2; % [deg]
zed_inc_y = 14.0375;
rotagl = [0,zed_inc_y,zed_inc_z];
translation = [0 0 0];
tform_lidar = rigidtform3d(rotagl,translation);

%% Load bag file
[file,path] = uigetfile("E:/nissan/*.bag");
bag = rosbag(string(path) + string(file));

%% Read topicks of pointcloud
% ros ouster-front pointcloud topic
% ros time resolution is 1e-6
disp("Loading ouster_front/points ...")
f_ousbag = select(bag,'Topic','/ouster_front/points');
f_ousMsgs = readMessages(f_ousbag);
f_ousts = timeseries(f_ousbag);
f_oust = f_ousts.Time;
disp("Successfuly loaded '/ouster_front/points' .")


% ros ouster-roof pointcloud topic
disp("Loading ouster_rear/points ...")
% ros time resolution is 1e-6
r_ousbag = select(bag,'Topic','/ouster_roof/points');
r_ousMsgs = readMessages(r_ousbag);
r_ousts = timeseries(r_ousbag);
r_oust = r_ousts.Time;
disp("Successfuly loaded '/ouster_roof/points' .")

% ros zed2i image topic
disp("Loading zed2i/zed_node/rgb/image_rect_color/compressed ...")
% ros time resolution is 1e-6
zedbag = select(bag,'Topic','/zed2i/zed_node/rgb/image_rect_color/compressed');
zedMsgs = readMessages(zedbag);
zedts = timeseries(zedbag);
zedt = zedts.Time;
disp("Successfuly loaded '/zed2i/zed_node/rgb/image_rect_color/compressed' .")

%% ros zed2i pointcloud topic

%% show
disp("start");
figure("Position",[100 100 1600 900]);
ax_img = subplot(1,2,1);
ax_pcd = subplot(1,2,2);
gridStep = 0.05;

% define start and end time
[min_len,min_idx] = min([length(r_oust),length(f_oust)]);
max_time = max([r_oust; f_oust],[],'all');
min_time = min([r_oust; f_oust],[],'all');

% define time step
dt = 1e-5;
time_list = min_time:dt:max_time;

% rounding
round_num = 3;
f_oust_rounded = round(f_oust,round_num);
r_oust_rounded = round(r_oust,round_num);
zedt_rounded   = round(zedt,round_num);

% Settings for calculate the number of datas been paied attended
start_sec = 133;
end_sec = 140;
start_idx = int32(start_sec/dt);
end_idx = int32(end_sec/dt);
front_pass_data_num = sum(f_oust_rounded < time_list(start_idx));
rear_pass_data_num = sum(r_oust_rounded < time_list(start_idx));
zed_pass_data_num = sum(zedt_rounded < time_list(start_idx));
f_oust_rounded = f_oust_rounded(f_oust_rounded > time_list(start_idx));
r_oust_rounded = r_oust_rounded(r_oust_rounded > time_list(start_idx));
zedt_rounded = zedt_rounded(zedt_rounded > time_list(start_idx));

f_data_num = front_pass_data_num + 1;
r_data_num = rear_pass_data_num + 1;
zed_data_num = zed_pass_data_num + 1;
f_oust_dnum = 1;
r_oust_dnum = 1;
zedt_dnum = 1;

all_around = false;
make_video = true;

if make_video
    save_dir = "video";
    file_name = file(1:end-4) + "_start-" + start_sec + "_end-" + end_sec;
    save_path = save_dir + "/" + file_name;
    v = VideoWriter(save_path + ".mp4", 'MPEG-4');
    v.Quality = 100;
    v.FrameRate = 10;
    open(v);
end

%% Loop
front_found_pcd = false;
rear_found_pcd = false;
last_f_ospc = 0;
last_r_ospc = 0;
for i = start_idx:end_idx
    
    % zed camera
    if abs(time_list(i) - zedt_rounded(zedt_dnum)) < dt
        % disp ouster-front pointcloud
        zed_img_read = rosReadImage(struct(zedMsgs{zed_data_num}));
        
        % size for applying Fusino net
        % zed_img_read(800-255:800,1920/2-1216/2+1:1920/2+1216/2,:)
        
        if zedt_dnum >= 2
            delete(zed_img_show);
        end
        zed_img_show = imshow(zed_img_read, 'Parent', ax_img);

        zed_data_num = zed_data_num + 1;
        zedt_dnum = zedt_dnum + 1;
        drawnow
    end

    % Front ploint clouds
    if abs(time_list(i) - f_oust_rounded(f_oust_dnum)) < dt
            front_found_pcd = true;
            % disp ouster-front pointcloud
            f_ospc_read = readXYZ(f_ousMsgs{f_data_num});
            % f_ospc = pointCloud(f_ospc_read(f_ospc_read(:,2)<=1.2 & f_ospc_read(:,2)>=-7 & f_ospc_read(:,1)>=-2 & f_ospc_read(:,1)<=2,:,:));
            % f_ospc = pointCloud(f_ospc_read(f_ospc_read(:,2)<=1.2 & f_ospc_read(:,2)>=-7,:,:));
            f_ospc = pointCloud(f_ospc_read);
            f_ospc = pctransform(f_ospc,f_tform);
            
            if ~all_around
                f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,1)>=1.2 & f_ospc.Location(:,2)>=-4 & f_ospc.Location(:,2)<=4 & f_ospc.Location(:,3)<=5,:,:));
                f_downptCloud = pcdownsample(f_ospc,'gridAverage',gridStep);
                [f_ospc, ~, plane_tform] = fitplane(f_ospc,f_downptCloud,0.01);
                f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,3)>=-1 & f_ospc.Location(:,3)<=0.2 & f_ospc.Location(:,2)>=-2 & f_ospc.Location(:,2)<=2 & f_ospc.Location(:,1)<=8.5,:,:));
            else
                f_ospc = pctransform(f_ospc,tform_lidar);
                xlim([-60,60]);
                ylim([-50,50]);
            end
            
            % if f_oust_dnum >= 2
            %     delete(f_ouspc_show);
            % end
            % % f_ospc = pctransform(f_ospc,tform_cam2wheel);
            % subplot(1,2,2);
            % f_ouspc_show = pcshow(f_ospc); hold on;
            % xlim([0,9]);
            % xlabel("\itX \rm[m]");
            % ylabel("\itY \rm[m]");
            % zlabel("\itZ \rm[m]");

            f_data_num = f_data_num + 1;
            f_oust_dnum = f_oust_dnum + 1;
            % view(-90,40)
            
            % fontname(gcf,"Times New Roman");
            % fontsize(gca,16,"points");
            % drawnow
    end

    % Roof point clouds
    if abs(time_list(i) - r_oust_rounded(r_oust_dnum)) < dt
            rear_found_pcd = true;
            % disp ouster-roof pointcloud
            r_ospc_read = readXYZ(r_ousMsgs{r_data_num}); % 130 -> bar
            % r_ospc = pointCloud(r_ospc_read(r_ospc_read(:,1)>=1.5 & r_ospc_read(:,1)<=8.5 & r_ospc_read(:,2)>=-2 & r_ospc_read(:,2)<=2,:,:));
            % r_ospc = pointCloud(r_ospc_read(r_ospc_read(:,1)>=1.5 & r_ospc_read(:,1)<=8.5,:,:));
            r_ospc = pointCloud(r_ospc_read);
            
            % r_ospc = pctransform(r_ospc,f_tform);  % from front-lidar to stereo-camera

            r_ospc = pctransform(r_ospc,r_tform);

            if ~all_around
                r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,1)>=1 & r_ospc.Location(:,2)>=-4 & r_ospc.Location(:,2)<=4 & r_ospc.Location(:,3)<=5,:,:));
                r_downptCloud = pcdownsample(r_ospc,'gridAverage',gridStep);
                [r_ospc, ~, plane_tform] = fitplane(r_ospc,r_downptCloud,0.01);
                r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,3)>=-1 & r_ospc.Location(:,3)<=0.2 & r_ospc.Location(:,2)>=-2 & r_ospc.Location(:,2)<=2 & r_ospc.Location(:,1)<=9,:,:));
            else
                r_ospc = pctransform(r_ospc,tform_lidar);
                xlim([-60,60]);
                ylim([-50,50]);
            end

            % if r_oust_dnum >= 2
            %     delete(r_ouspc_show);
            % end
            % % r_ospc = pctransform(r_ospc,tform_cam2wheel);
            % subplot(1,2,2);
            % r_ouspc_show = pcshow(r_ospc); hold on;
            % xlim([0,9]);
            % xlabel("\itX \rm[m]");
            % ylabel("\itY \rm[m]");
            % zlabel("\itZ \rm[m]");

            r_data_num = r_data_num + 1;
            r_oust_dnum = r_oust_dnum + 1;
            % view(-90,40)

            % fontname(gcf,"Times New Roman");
            % fontsize(gca,16,"points");

            % set(gcf,'color','w');
            % set(gca,'color','w');
            % set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
    end
    if front_found_pcd & rear_found_pcd & any([f_ospc~=last_f_ospc, r_ospc~=last_r_ospc])
        if f_oust_dnum >= 2
            delete(f_ouspc_show);
        end
        % f_ospc = pctransform(f_ospc,tform_cam2wheel);
        subplot(1,2,2);
        f_ouspc_show = pcshow(f_ospc); hold on;

        if r_oust_dnum >= 2
            delete(r_ouspc_show);
        end
        % r_ospc = pctransform(r_ospc,tform_cam2wheel);
        subplot(1,2,2);
        r_ouspc_show = pcshow(r_ospc); hold on;
        xlim([0,9]);
        view(-90,40)
        xlabel("\itX \rm[m]");
        ylabel("\itY \rm[m]");
        zlabel("\itZ \rm[m]");
        fontname(gcf,"Times New Roman");
        fontsize(gca,16,"points");

        drawnow
        last_f_ospc = f_ospc;
        last_r_ospc = r_ospc;
    end
    disp(round(time_list(i)-time_list(1),2)+"[s]--"+ f_data_num + " " + r_data_num);
    if make_video && mod(time_list(i)-time_list(1),dt*10) == 0
        frame = getframe(gcf);
        writeVideo(v,frame);
    end

    % disp zed2i pointcloud
    % zedpc = pointCloud(readXYZ(zedMsgs{i}));
    % zedpc = pctransform(zedpc,tform);
    % zedpc = pctransform(zedpc,tform_lidar);
    % zedpc_show = pcshow(zedpc);

    % delete(zedpc_show);
    % n_strPadded = sprintf('%04d',i) ;
    % pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    % pcwrite(pc,pcFileName);
end
if make_video
    close(v)
end

% pclist = dir("C:\Users\masato\AppData\Local\Temp\PointClouds\*.pcd");
% for i = 1:length(pclist)
%     filename = pclist(i).folder + "/" + pclist(i).name;
%     ptcloud = pcread(filename);
%     pcshow(ptcloud);
%     drawnow
% end

%% ego view
% helperVisualizeEgoView(pointCloud(f_ospc_read));
%%%
% *|helperVisualizeEgoView|* visualizes point cloud data in the ego
% perspective by rotating about the center.
function player = helperVisualizeEgoView(ptCloud)

% Create a pcplayer object
xlimits = ptCloud.XLimits;
ylimits = ptCloud.YLimits;
zlimits = ptCloud.ZLimits;

player = pcplayer(xlimits, ylimits, zlimits);

% Turn off axes lines
axis(player.Axes, 'off');

% Set up camera to show ego view
camproj(player.Axes, 'perspective');
camva(player.Axes, 90);
campos(player.Axes, [0 0 0]);
camtarget(player.Axes, [-1 0 0]);

% Set up a transformation to rotate by 5 degrees
theta = 5;

eulerAngles = [0 0 theta];
translation = [0 0 0];
rotateByTheta = rigidtform3d(eulerAngles, translation);

for n = 0 : theta : 359
    % Rotate point cloud by theta
    ptCloud = pctransform(ptCloud, rotateByTheta);
    
    % disp point cloud
    view(player, ptCloud);
    
    pause(0.05)
end
end