function f_prev_profile = func__ptc2profile(ptc)
%% set infomations
msg_nums = [2748 2750];
gridStep = 0.05;

% install params
f_ousmsg_num = msg_nums(1);
r_ousmsg_num = msg_nums(2);

%% correct road surface profile
% max_z0 = 0.025;                                                                % [m] max road displacement
% ld = [0.05 0.15 0.05];
% start_disturbance = 3; % 2.96                                                                  % amplitude
% max_distance = 30;                                                           % [m] driving mileage
% f_dis_total = [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
% r_dis_total =  [0,start_disturbance,start_disturbance+ld(1),start_disturbance+sum(ld(1:2)),start_disturbance+sum(ld),max_distance];
% road_total = [0,0,max_z0,max_z0,0,0];  % converting front disturbance and buffer ([m])

%% 2d profile
% figure("Position", [100 50 260 145]);
% figure("Position", [100 50 260 340/2]);
range_min = 3;        % minimum measurable distance [m]
range_max = 5.5;        % maximum measurable distance [m]
pick_up_width = 0.7;  % width of datas for a road profile [m]
pick_up_center = 0;   % center of pick up position [m]
p_min = pick_up_center - pick_up_width/2;
p_max = pick_up_center + pick_up_width/2;

% prediction
f_ospc = ptc;
f_line = f_ospc.Location(f_ospc.Location(:,2)>=p_min & f_ospc.Location(:,2)<=p_max & f_ospc.Location(:,1)<=range_max & f_ospc.Location(:,1)>=range_min,:,:);
% f_line = f_ospc.Location(f_ospc.Location(:,1)>=-0.075 & f_ospc.Location(:,1)<=0.075 & f_ospc.Location(:,2)<=7 & f_ospc.Location(:,2)>=5.06,:,:);
[~,f_ind] = sort(f_line(:,1));
f_prev_profile=f_line(f_ind,[true false true])';

end
