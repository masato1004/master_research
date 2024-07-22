%% rpf noising
% road_total_f(isnan(road_total_f))=0;
% road_total_r(isnan(road_total_r))=0;
% noised_road_total_f = awgn(movmean(road_total_f,301),0.01,'measured');
% noised_road_total_r = awgn(movmean(road_total_r,301),0.01,'measured');
% noised_r_p = [
%     noised_road_total_f;
%     noised_road_total_r;
%     gradient(noised_road_total_f)./dt;
%     gradient(noised_road_total_r)./dt
%     ];
% save("noised_r_p","noised_r_p");
load("noised_r_p");


%% Calculate wheel trajectory with inverse offset method
% noised_inv_offset_f = [];
% noised_inv_offset_r = [];
% disp('Calculating inverse offset from noised data...')
% for i = 1:1:width(dis_total)
%     current_road_f = dis_total(dis_total > dis_total(i)-r & dis_total < dis_total(i)+r);
%     current_road_f = current_road_f(1:5:end);
%     half_circle_f = [current_road_f; sqrt(r^2 - (current_road_f-dis_total(i)).^2)+noised_road_total_f(i)-r];
%     noised_inv_offset_f = [noised_inv_offset_f, half_circle_f];
% 
%     current_road_r = dis_total(dis_total > dis_total(i)-r & dis_total < dis_total(i)+r);
%     current_road_r = current_road_r(1:5:end);
%     half_circle_r = [current_road_r; sqrt(r^2 - (current_road_r-dis_total(i)).^2)+noised_road_total_r(i)-r];
%     noised_inv_offset_r = [noised_inv_offset_r, half_circle_r];
% end
% save("noised_inverse_offset-"+shape, "noised_inv_offset_f", "noised_inv_offset_r")
% disp('Done Calculating inverse offset.')
% 
% noised_wheel_traj_f = [];
% noised_wheel_traj_r= [];
% disp('Calculating wheel trajectory...')
% load("noised_inverse_offset-"+shape+".mat")
% for i = 1:10:width(dis_total)
%     if i == 1
%         current_traj_f = [dis_total(i); max(noised_inv_offset_f(2,noised_inv_offset_f(1,:)==dis_total(i) | noised_inv_offset_f(1,:)==dis_total(i+1)))];
%         current_traj_r = [dis_total(i); max(noised_inv_offset_r(2,noised_inv_offset_r(1,:)==dis_total(i) | noised_inv_offset_r(1,:)==dis_total(i+1)))];
%     elseif i == width(dis_total)
%         current_traj_f = [dis_total(i); max(noised_inv_offset_f(2,noised_inv_offset_f(1,:)==dis_total(i-1) | noised_inv_offset_f(1,:)==dis_total(i)))];
%         current_traj_r = [dis_total(i); max(noised_inv_offset_r(2,noised_inv_offset_r(1,:)==dis_total(i-1) | noised_inv_offset_r(1,:)==dis_total(i)))];
%     else
%         current_traj_f = [dis_total(i); max(noised_inv_offset_f(2,noised_inv_offset_f(1,:)==dis_total(i-1) | noised_inv_offset_f(1,:)==dis_total(i) | noised_inv_offset_f(1,:)==dis_total(i+1)))];
%         current_traj_r = [dis_total(i); max(noised_inv_offset_r(2,noised_inv_offset_r(1,:)==dis_total(i-1) | noised_inv_offset_r(1,:)==dis_total(i) | noised_inv_offset_r(1,:)==dis_total(i+1)))];
%     end
%     noised_wheel_traj_f = [noised_wheel_traj_f, current_traj_f];
%     noised_wheel_traj_r = [noised_wheel_traj_r, current_traj_r];
% end
% noised_wheel_traj_f = [dis_total; makima(noised_wheel_traj_f(1,:),noised_wheel_traj_f(2,:),dis_total)];
% noised_wheel_traj_r = [dis_total; makima(noised_wheel_traj_r(1,:),noised_wheel_traj_r(2,:),dis_total)];
% save("noised_wheel_traj-"+shape, "noised_wheel_traj_f", "noised_wheel_traj_r")
% disp('Done Calculating wheel trajectory.')
% 
% disp('Calculating wheel mileage...')
% noised_wheel_traj_f(2,abs(noised_wheel_traj_f(2,:)) < 0.000005) = 0;
% noised_wheel_traj_r(2,abs(noised_wheel_traj_r(2,:)) < 0.000005) = 0;
% noised_mileage_f = 0;
% noised_mileage_r = 0;
% for k = 2:width(noised_wheel_traj_f)
%     noised_mileage_f = [noised_mileage_f, sum(sqrt(diff(noised_wheel_traj_f(1,1:k)).^2+diff(noised_wheel_traj_f(2,1:k)).^2))];
%     noised_mileage_r = [noised_mileage_r, sum(sqrt(diff(noised_wheel_traj_r(1,1:k)).^2+diff(noised_wheel_traj_r(2,1:k)).^2))];
% end
% save('noised_mileage','noised_mileage_f','noised_mileage_r');
% disp('Done Calculating wheel mileage.')

%% Load wheel trajectory
load("noised_wheel_traj-"+shape+".mat");

%% Load wheel mileage
load("noised_mileage.mat");