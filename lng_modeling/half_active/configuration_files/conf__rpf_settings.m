warning("off","all");
preT = 10;
dis = 0:preT*V/(preT/dt):preT*V;                                                       % distance time line ([m])
start_disturbance = 9;                                                        % the start distance of disturbance ([m])

max_z0 = 0.025;                                                                % [m] max road displacement
ld = 0.25;
const = 6;                                                                    % amplitude
max_distance = 100;                                                           % [m] driving mileage
dis_total = 0:max_distance/(preT/dt):max_distance;                               % distance list for road profile ([m])
dis_total_f = 0:max_distance/(preT/dt):max_distance-start_disturbance;                           % distance list for front road profile ([m])
dis_total_r = 0:max_distance/(preT/dt):max_distance-start_disturbance-(L_f+L_r);                 % distance list for rear road profile ([m])

[road_total_f,road_total_r,ld,frequency,max_z0,dis_length] = road_prof_maker(shape,TL,preT,dt,V,L_f,L_r,dis,start_disturbance,max_z0,ld,const,max_distance,dis_total,dis_total_f,dis_total_r);

% if sensing
%     r_p_f = interp1(road_total_f(:,1)',road_total_f(:,2)',dis,'linear');                                   % front road profile
%     r_p_r = interp1(road_total_r(:,1)',road_total_r(:,2)',dis,'linear');                                   % rear road profile
%     r_p_f = lowpass(r_p_f,0.5,1/dt);
%     r_p_r = lowpass(r_p_r,0.5,1/dt);
% else
%     r_p_f = makima(dis_total,road_total_f,dis);                                   % front road profile
%     r_p_r = makima(dis_total,road_total_r,dis);                                   % rear road profile
% end


%% Load wheel trajectory
load("configuration_files/wheel_traj-"+shape+".mat");

%% Calculate wheel trajectory with inverse offset method
inv_offset_f = [];
inv_offset_r = [];
disp('Calculating inverse offset...')
for i = 1:1:width(dis_total)
    current_road_f = dis_total(dis_total > dis_total(i)-r & dis_total < dis_total(i)+r);
    current_road_f = current_road_f(1:5:end);
    half_circle_f = [current_road_f; sqrt(r^2 - (current_road_f-dis_total(i)).^2)+road_total_f(i)-r];
    inv_offset_f = [inv_offset_f, half_circle_f];

    current_road_r = dis_total(dis_total > dis_total(i)-r & dis_total < dis_total(i)+r);
    current_road_r = current_road_r(1:5:end);
    half_circle_r = [current_road_r; sqrt(r^2 - (current_road_r-dis_total(i)).^2)+road_total_r(i)-r];
    inv_offset_r = [inv_offset_r, half_circle_r];
end
disp('Done Calculating inverse offset.')

wheel_traj_f = [];
wheel_traj_r= [];
disp('Calculating wheel trajectory...')
for i = 1:1:width(dis_total)
    if i == 1
        current_traj_f = [dis_total(i); max(inv_offset_f(2,inv_offset_f(1,:)==dis_total(i) | inv_offset_f(1,:)==dis_total(i+1)))];
        current_traj_r = [dis_total(i); max(inv_offset_r(2,inv_offset_r(1,:)==dis_total(i) | inv_offset_r(1,:)==dis_total(i+1)))];
    elseif i == width(dis_total)
        current_traj_f = [dis_total(i); max(inv_offset_f(2,inv_offset_f(1,:)==dis_total(i-1) | inv_offset_f(1,:)==dis_total(i)))];
        current_traj_r = [dis_total(i); max(inv_offset_r(2,inv_offset_r(1,:)==dis_total(i-1) | inv_offset_r(1,:)==dis_total(i)))];
    else
        current_traj_f = [dis_total(i); max(inv_offset_f(2,inv_offset_f(1,:)==dis_total(i-1) | inv_offset_f(1,:)==dis_total(i) | inv_offset_f(1,:)==dis_total(i+1)))];
        current_traj_r = [dis_total(i); max(inv_offset_r(2,inv_offset_r(1,:)==dis_total(i-1) | inv_offset_r(1,:)==dis_total(i) | inv_offset_r(1,:)==dis_total(i+1)))];
    end
    wheel_traj_f = [wheel_traj_f, current_traj_f];
    wheel_traj_r = [wheel_traj_r, current_traj_r];
end
wheel_traj_f = [dis_total; makima(wheel_traj_f(1,:),wheel_traj_f(2,:),dis_total)];
wheel_traj_r = [dis_total; makima(wheel_traj_r(1,:),wheel_traj_r(2,:),dis_total)];
save("wheel_traj-"+shape, "wheel_traj_f", "wheel_traj_r")
disp('Done Calculating wheel trajectory.')

disp('Calculating wheel mileage...')
wheel_traj_f(2,abs(wheel_traj_f(2,:)) < 0.000005) = 0;
wheel_traj_r(2,abs(wheel_traj_r(2,:)) < 0.000005) = 0;
mileage_f = 0;
mileage_r = 0;
for k = 2:width(wheel_traj_f)
    mileage_f = [mileage_f, sum(sqrt(diff(wheel_traj_f(1,1:k)).^2+diff(wheel_traj_f(2,1:k)).^2))];
    mileage_r = [mileage_r, sum(sqrt(diff(wheel_traj_r(1,1:k)).^2+diff(wheel_traj_r(2,1:k)).^2))];
end
save('mileage','mileage_f','mileage_r');
disp('Done Calculating wheel mileage.')

load("mileage.mat");

r_p = [
    road_total_f;
    road_total_r;
    gradient(road_total_f)./dt;
    gradient(road_total_r)./dt
    ];
% r_p_prev = [
%     dis;
%     r_p_f(end-144:end),r_p_f(1:end-145);
%     r_p_r(end-144:end),r_p_r(1:end-145);
%     gradient([r_p_f(end-144:end),r_p_f(1:end-145)])./dt;
%     gradient([r_p_r(end-144:end),r_p_r(1:end-145)])./dt
%     ];
r_p_prev = [
    dis;
    road_total_f;
    road_total_r;
    gradient(road_total_f)./dt;
    gradient(road_total_r)./dt
    ];