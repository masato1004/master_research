dis = 0:T*V/(T/dt):T*V;                                                       % distance time line ([m])
start_disturbance = 9;                                                        % the start distance of disturbance ([m])

max_z0 = 0.025;                                                                % [m] max road displacement
ld = 0.25;
const = 6;                                                                    % amplitude
max_distance = 100;                                                           % [m] driving mileage
dis_total = 0:max_distance/(T/dt):max_distance;                               % distance list for road profile ([m])
dis_total_f = 0:max_distance/(T/dt):max_distance-start_disturbance;                           % distance list for front road profile ([m])
dis_total_r = 0:max_distance/(T/dt):max_distance-start_disturbance-(L_f+L_r);                 % distance list for rear road profile ([m])

[road_total_f,road_total_r,ld,frequency,max_z0,dis_length] = road_prof_maker(shape,TL,T,dt,V,L_f,L_r,dis,start_disturbance,max_z0,ld,const,max_distance,dis_total,dis_total_f,dis_total_r);

% if sensing
%     r_p_f = interp1(road_total_f(:,1)',road_total_f(:,2)',dis,'linear');                                   % front road profile
%     r_p_r = interp1(road_total_r(:,1)',road_total_r(:,2)',dis,'linear');                                   % rear road profile
%     r_p_f = lowpass(r_p_f,0.5,1/dt);
%     r_p_r = lowpass(r_p_r,0.5,1/dt);
% else
%     r_p_f = makima(dis_total,road_total_f,dis);                                   % front road profile
%     r_p_r = makima(dis_total,road_total_r,dis);                                   % rear road profile
% end

% inverse offset method for calculate wheel trajectory
inv_offset_f = [];
for i = 1:5:width(dis_total)
    current_road_f = dis_total(dis_total > dis_total(i)-r & dis_total < dis_total(i)+r);
    current_road_f = decimate(current_road_f,6);
    half_circle_f = [current_road_f; sqrt(r^2 - (current_road_f-dis_total(i)).^2)+road_total_f(i)-r];
    inv_offset_f = [inv_offset_f, half_circle_f];
end

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