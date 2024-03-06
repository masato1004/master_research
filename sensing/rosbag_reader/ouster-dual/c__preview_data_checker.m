close all;
warning('off','all');
%% Vlocity settings
% v20
velocity = 19.05;
msg_nums_start = [948 951];
msg_nums_end = [971 974];

% v15
% velocity = 14.1;
% msg_nums_start = [1824 1827];
% msg_nums_end = [1854 1857];

% v10
% velocity = 11.1;
% msg_nums_start = [2722 2725];
% msg_nums_end = [2760 2763];

% define params
f_ousmsg_num_start = msg_nums_start(1);
r_ousmsg_num_start = msg_nums_start(2);
f_ousmsg_num_end = msg_nums_end(1);
r_ousmsg_num_end = msg_nums_end(2);

%% Downsample settings for esimate road surface plane
gridStep = 0.05;

%% Channel number settings
low_channel = false; % choose from [32,64,false(=128)]

channel_step = 128/low_channel;
min_rad = deg2rad(-22.5);
max_rad = deg2rad(22.5);
diff_rad = deg2rad(45/128);

%% correct road surface profile
max_z0 = 0.025;                                                                % [m] max road displacement
ld = [0.049 0.15 0.049];
start_disturbance = 6.95;                                                                    % amplitude
max_distance = 30;                                                           % [m] driving mileage
f_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
r_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
road_total = [0,0,max_z0,max_z0,0,0];  % converting front disturbance and buffer ([m])

%% 2 dimentionize settings
range_min = 0;        % minimum measurable distance [m]
range_max = 8;        % maximum measurable distance [m]
pick_up_width = 0.2;  % width of datas for a road profile [m]
pick_up_center = 0;   % center of pick up position [m]

% movmean setting
mean_data_num = [30, 30];

% define params
p_min = pick_up_center - pick_up_width/2;
p_max = pick_up_center + pick_up_width/2;

%% video settings
save_video = false;

if save_video
    if ~low_channel
        video_dir = "video/128/pickup_width-"+pick_up_width;
    else
        video_dir = "video/"+low_channel+"/pickup_width-"+pick_up_width;
    end
    folder_maker(video_dir);
    videoname = "v" + velocity;
    video = VideoWriter(video_dir + "/" + videoname,'MPEG-4');
    video.FrameRate = 2;
    open(video);
end

%% figure settings
fig = figure('name', "RSP from LiDAR",'Position', [100 100 700 380]);

% time count
timedata = 0;
str = {timedata + " [s]"};

% front
f_ax = subplot(211);% front
f_correct_road = plot(f_dis_total,road_total,"LineWidth",2,"Color","#aaaaaa"); hold on;
f_sc = scatter(0,0,1.2,"filled","MarkerFaceColor","#0000ff"); hold on;  % picked up points
f_pl = plot(0,0,"LineWidth",2,"LineStyle",":","Color","#ff0000"); % moving average
grid on;
xlim(f_ax,[range_min,range_max]);
ylim(f_ax,[-0.08, 0.08]);
% axis equal;
f_time_text = text(7,0.05,str);
f_time_text.FontSize = 10;
xlabel(f_ax,"Local Distance from Front Wheel [m]");
ylabel(f_ax,"Previewed Displacement [m]");
title(f_ax,"Front LiDAR");
legend(["Actual Road","Raw data","Moving Average"],"Location","southwest");

% rear
r_ax = subplot(212);
r_correct_road = plot(r_dis_total,road_total,"LineWidth",2,"Color","#aaaaaa"); hold on;
r_sc = scatter(0,0,1.2,"filled","MarkerFaceColor","#0000ff"); % picked up points
r_pl = plot(0,0,"LineWidth",2,"LineStyle",":","Color","#ff0000"); % moving average
grid on;
xlim(r_ax,[range_min,range_max]);
ylim(r_ax,[-0.08, 0.08]);
% axis equal;
r_time_text = text(7,0.05,str);
r_time_text.FontSize = 10;
xlabel(r_ax,"Local Distance from Front Wheel [m]");
ylabel(r_ax,"Previewed Displacement [m]");
title(r_ax,"Roof LiDAR");
legend(["Actual Road","Raw data","Moving Average"],"Location","southwest");

fontname(gcf,"Times New Roman");
fontsize(gcf,10,"points");

% drawnow;

%% loop settings
data_num = min(abs(msg_nums_start- msg_nums_end));

ts = 0.05;
V = velocity*1000/3600;

f_last_data_num = f_ousmsg_num_start;
r_last_data_num = r_ousmsg_num_start;

f_error = {};
r_error = {};

%% loop
tic
for i = 0:data_num

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % get point cloud and transfrom into front-wheel-coordinate

    % front
    f_ospc_read = readXYZ(f_ousMsgs{f_ousmsg_num_start+i});
    % simulate lower channel lidar
    if low_channel
        [~,rho,z] = cart2pol(f_ospc_read(:,1),f_ospc_read(:,2),f_ospc_read(:,3));
        phi = atan2(z, rho);
        [phi,phi_idx] = sort(phi);
        f_data = f_ospc_read(phi_idx,:);
        f_data32 = [];
        for k = 0:channel_step:127
            f_data32 = [f_data32; f_data(phi >= min_rad+k*diff_rad & phi < min_rad+(k+1)*diff_rad,:)];
        end

        f_ospc = pointCloud(f_data32);
    else
        f_ospc = pointCloud(f_ospc_read);
    end
    
    f_ospc = pctransform(f_ospc,f_tform);
    f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,1)>=1.2 & f_ospc.Location(:,2)>=-3 & f_ospc.Location(:,2)<=3 & f_ospc.Location(:,3)<=5,:,:));
    f_downptCloud = pcdownsample(f_ospc,'gridAverage',gridStep);
    [f_ospc, ~, plane_tform] = fitplane(f_ospc,f_downptCloud,0.01);
    f_osdata = f_ospc.Location(f_ospc.Location(:,3)>=-1 & f_ospc.Location(:,3)<=0.2 & f_ospc.Location(:,2)>=-2 & f_ospc.Location(:,2)<=2 & f_ospc.Location(:,1)<=8.5,:,:);
    % f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,3)>=-1 & f_ospc.Location(:,3)<=0.2 & f_ospc.Location(:,2)>=-2 & f_ospc.Location(:,2)<=2 & f_ospc.Location(:,1)<=8.5,:,:));
    % f_ospc = pctransform(f_ospc,tform_cam2wheel);

    % roof
    r_ospc_read = readXYZ(r_ousMsgs{r_ousmsg_num_start+i}); % 965 -> hump
    % simulate lower channel lidar
    if low_channel
        [~,rho,z] = cart2pol(r_ospc_read(:,1),r_ospc_read(:,2),r_ospc_read(:,3));
        phi = atan2(z, rho);
        [phi,phi_idx] = sort(phi);
        r_data = r_ospc_read(phi_idx,:);
        r_data32 = [];
        for k = 0:channel_step:127
            r_data32 = [r_data32; r_data(phi >= min_rad+k*diff_rad & phi < min_rad+(k+1)*diff_rad,:)];
        end

        r_ospc = pointCloud(r_data32);
    else
        r_ospc = pointCloud(r_ospc_read);
    end

    r_ospc = pctransform(r_ospc,r_tform);
    r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,1)>=1 & r_ospc.Location(:,2)>=-4 & r_ospc.Location(:,2)<=4 & r_ospc.Location(:,3)<=5,:,:));
    r_downptCloud = pcdownsample(r_ospc,'gridAverage',gridStep);
    [r_ospc, ~, plane_tform] = fitplane(r_ospc,r_downptCloud,0.01);
    r_osdata = r_ospc.Location(r_ospc.Location(:,3)>=-1 & r_ospc.Location(:,3)<=0.2 & r_ospc.Location(:,2)>=-2 & r_ospc.Location(:,2)<=2 & r_ospc.Location(:,1)<=9,:,:);
    % r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,3)>=-1 & r_ospc.Location(:,3)<=0.2 & r_ospc.Location(:,2)>=-2 & r_ospc.Location(:,2)<=2 & r_ospc.Location(:,1)<=9,:,:));
    % r_ospc = pctransform(r_ospc,tform_cam2wheel);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 2 dimentionize
    % front
    f_line = f_osdata(f_osdata(:,2)>=p_min & f_osdata(:,2)<=p_max & f_osdata(:,1)<=range_max & f_osdata(:,1)>=range_min,:,:);
    [~,f_ind] = sort(f_line(:,1));
    f_prev_profile=f_line(f_ind,[true false true])';
    
    % roof
    r_line = r_osdata(r_osdata(:,2)>=p_min & r_osdata(:,2)<=p_max & r_osdata(:,1)<=range_max & r_osdata(:,1)>=range_min,:,:);
    % r_line = r_ospc.Location(r_ospc.Location(:,1)>=-0.075 & r_ospc.Location(:,1)<=0.075 & r_ospc.Location(:,2)<=7 & r_ospc.Location(:,2)>=5.06,:,:);
    [~,r_ind] = sort(r_line(:,1));
    r_prev_profile=r_line(r_ind,[true false true])';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % drawing
    timedata = timedata + ts;
    str = {timedata + " [s]"};
    f_time_text.String = str;
    r_time_text.String = str;

    set(f_sc,"XData",f_prev_profile(1,:),"YData",f_prev_profile(2,:));
    set(f_pl,"XData",f_prev_profile(1,:),"YData",movmean(f_prev_profile(2,:),mean_data_num));
    set(r_sc,"XData",r_prev_profile(1,:),"YData",r_prev_profile(2,:));
    set(r_pl,"XData",r_prev_profile(1,:),"YData",movmean(r_prev_profile(2,:),mean_data_num));

    % correct road surface profile
    % front
    f_dis_total = f_dis_total - V*(f_oust(f_ousmsg_num_start+i) - f_oust(f_last_data_num));
    set(f_correct_road,"XData",f_dis_total);
    f_last_data_num = f_ousmsg_num_start+i;

    % roof
    r_dis_total = r_dis_total - V*(r_oust(r_ousmsg_num_start+i) - r_oust(r_last_data_num));
    set(r_correct_road,"XData",r_dis_total);
    r_last_data_num = r_ousmsg_num_start+i;

    pause(0.5);

    drawnow;
    if save_video
        frame = getframe(fig);
        writeVideo(video,frame);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % calculate Error between actual road and estimated road
    % front
    f_dis_total_p = [f_dis_total, f_prev_profile(1,:)];
    [f_dis_total_p,f_dis_idx] = sort(f_dis_total_p);
    f_correct_road_p = interp1(f_dis_total,road_total,f_dis_total_p);

    f_error{i+1} = [f_prev_profile(1,:); movmean(f_prev_profile(2,:),mean_data_num) - f_correct_road_p(ismember(f_dis_total_p, f_prev_profile(1,:)))];

    % roof
    r_dis_total_p = [r_dis_total, r_prev_profile(1,:)];
    [r_dis_total_p,r_dis_idx] = sort(r_dis_total_p);
    r_correct_road_p = interp1(r_dis_total,road_total,r_dis_total_p);

    r_error{i+1} = [r_prev_profile(1,:); movmean(r_prev_profile(2,:),mean_data_num) - r_correct_road_p(ismember(r_dis_total_p, r_prev_profile(1,:)))];

end
finish_time = toc
calculation_time_per_frame = finish_time/(data_num+1)

figure();
for i = 1:length(f_error)
    scatter(f_error{i}(1,:),f_error{i}(2,:),1.2,"filled");hold on; grid on;
end

if save_video
    close(video);
end