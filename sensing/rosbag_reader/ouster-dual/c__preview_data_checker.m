close all;
%% set infomations
% v20
velocity = 20;
msg_nums_start = [949 951];
msg_nums_end = [971 973];

% v15
% velocity = 14.1;
% msg_nums_start = [1824 1827];
% msg_nums_end = [1854 1857];

% v10
% velocity = 11;
% msg_nums_start = [2722 2724];
% msg_nums_end = [2760 2762];

gridStep = 0.05;

% define params
f_ousmsg_num_start = msg_nums_start(1);
r_ousmsg_num_start = msg_nums_start(2);
f_ousmsg_num_end = msg_nums_end(1);
r_ousmsg_num_end = msg_nums_end(2);

%% correct road surface profile
max_z0 = 0.025;                                                                % [m] max road displacement
ld = [0.05 0.15 0.05];
start_disturbance = 6.95;                                                                    % amplitude
max_distance = 30;                                                           % [m] driving mileage
f_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
r_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
road_total = [0,0,max_z0,max_z0,0,0];  % converting front disturbance and buffer ([m])

%% 2 dimentionize settings
range_min = 0;        % minimum measurable distance [m]
range_max = 8;        % maximum measurable distance [m]
pick_up_width = 0.3;  % width of datas for a road profile [m]
pick_up_center = 0;   % center of pick up position [m]

% movmean setting
mean_data_num = [50, 50];

% define params
p_min = pick_up_center - pick_up_width/2;
p_max = pick_up_center + pick_up_width/2;

%% video settings
video_dir = "video";
folder_maker(video_dir);
videoname = "v" + velocity;
video = VideoWriter(video_dir + "/" + videoname,'MPEG-4');
video.FrameRate = 1;
open(video);

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

all_list = [];

%% loop
tic
for i = 0:data_num

    % get point cloud and transfrom into front-wheel-coordinate
    % front
    f_ospc_read = readXYZ(f_ousMsgs{f_ousmsg_num_start+i});
    f_ospc = pointCloud(f_ospc_read);
    f_ospc = pctransform(f_ospc,f_tform);
    f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,1)>=1.2 & f_ospc.Location(:,2)>=-4 & f_ospc.Location(:,2)<=4 & f_ospc.Location(:,3)<=5,:,:));
    f_downptCloud = pcdownsample(f_ospc,'gridAverage',gridStep);
    [f_ospc, ~, plane_tform] = fitplane(f_ospc,f_downptCloud,0.01);
    f_ospc = pointCloud(f_ospc.Location(f_ospc.Location(:,3)>=-1 & f_ospc.Location(:,3)<=0.2 & f_ospc.Location(:,2)>=-2 & f_ospc.Location(:,2)<=2 & f_ospc.Location(:,1)<=8.5,:,:));
    % f_ospc = pctransform(f_ospc,tform_cam2wheel);

    % rear
    r_ospc_read = readXYZ(r_ousMsgs{r_ousmsg_num_start+i}); % 965 -> hump
    r_ospc = pointCloud(r_ospc_read);
    r_ospc = pctransform(r_ospc,r_tform);
    r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,1)>=1 & r_ospc.Location(:,2)>=-4 & r_ospc.Location(:,2)<=4 & r_ospc.Location(:,3)<=5,:,:));
    r_downptCloud = pcdownsample(r_ospc,'gridAverage',gridStep);
    [r_ospc, ~, plane_tform] = fitplane(r_ospc,r_downptCloud,0.01);
    r_ospc = pointCloud(r_ospc.Location(r_ospc.Location(:,3)>=-1 & r_ospc.Location(:,3)<=0.2 & r_ospc.Location(:,2)>=-2 & r_ospc.Location(:,2)<=2 & r_ospc.Location(:,1)<=9,:,:));
    % r_ospc = pctransform(r_ospc,tform_cam2wheel);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 2 dimentionize
    % front
    f_line = f_ospc.Location(f_ospc.Location(:,2)>=p_min & f_ospc.Location(:,2)<=p_max & f_ospc.Location(:,1)<=range_max & f_ospc.Location(:,1)>=range_min,:,:);
    [~,f_ind] = sort(f_line(:,1));
    f_prev_profile=f_line(f_ind,[true false true])';
    
    % rear
    r_line = r_ospc.Location(r_ospc.Location(:,2)>=p_min & r_ospc.Location(:,2)<=p_max & r_ospc.Location(:,1)<=range_max & r_ospc.Location(:,1)>=range_min,:,:);
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

    % rear
    r_dis_total = r_dis_total - V*(r_oust(r_ousmsg_num_start+i) - r_oust(r_last_data_num));
    set(r_correct_road,"XData",r_dis_total);
    r_last_data_num = r_ousmsg_num_start+i;

    pause(0.5);

    drawnow;
    frame = getframe(fig);
    writeVideo(video,frame);
    % delete(f_sc);
    % delete(f_pl);
    % delete(r_sc);
    % delete(r_pl);
end
finish_time = toc
calculation_time_per_frame = finish_time/(data_num+1)
close(video);