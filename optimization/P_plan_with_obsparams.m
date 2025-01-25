%% read datas
% close all;
% figure(Position=[680,458,560,420*2/3])
figure()


current_input = 0;
current_du = 0;
last_obs = [];
numClusters = 1;
obstacle_radius = 0.25;
height_obs = 0.02;
vel_list = [40, 50, 60];

% Define the results list
max_lateral_force_list = [];
distance_to_obstacle_list = [];

animation = false;
plot_sol = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  NLMPC code block
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pause(2)
% import casadi.*

% Clear and close all
% clear;
% close all;

for lp = 1:length(vel_list)
    % Define the vehicle parameters
    tw = 1.485; % Track width
    wb = 2.86; % Wheelbase
    vkm = vel_list(lp); % Velocity in km/h
    v = vkm/3.6; % Constant velocity
    L = wb; % Wheelbase
    mass = 1500;
    Iz = 2066;
    next_state = [0; 0; 0; v; 0; 0];
    
    % obstacle = [13.6512; 0.482284];
    lane_width = 3.5; % Width of the lane
    g = 9.8;
    
    % Define the obstacle parameters
    x_obs_list = 6:2*obstacle_radius:20;
    y_obs_list = -lane_width/2+obstacle_radius/2:0.1:lane_width/2-obstacle_radius/2;
    
    % Define the loop number by the number of obstacles
    loop_num = length(x_obs_list)*length(y_obs_list);

    over_flag = false;
    
    
    % Define the video object
    if animation
        video_name = vkm+"kmNLMPC_with_obs.mp4";
        video = VideoWriter(video_name,'MPEG-4');
        video.FrameRate = 50;
        open(video);
    end

    
    subplot(3,1,lp)
    text(23,1,vkm+" km/h",'FontSize',10,'HorizontalAlignment','center'); hold on;
    xline(wb,"--k","Front"+newline+"Wheels",'LabelHorizontalAlignment','right','LabelVerticalAlignment','bottom','LabelOrientation','aligned')
    for ii=1:length(x_obs_list)
        for jj=1:length(y_obs_list)
            x_obs = x_obs_list(end+1-ii);
            y_obs = y_obs_list(jj);
    
    
            % Define the optimization variables
            opti = casadi.Opti();
            N = 9; % Number of steps
            interp_steps = 2;
            x = opti.variable(6, N+1); % State variables (x, y, theta)
            u = opti.variable(1, N);   % Control variables
            du = opti.variable(1, N); % Change in control variables
            current_input = 0;
        
            % Define the dynamics
            dt = 0.15;
        
            % Define reference trajectory
            x_ref = [0:dt*v:dt*v*N] + wb/2;
            y_ref = zeros(size(x_ref));
            % y_ref = -(exp(x_ref*0.02) - 1);
            yaw_ref = atan2(gradient(y_ref), gradient(x_ref));
            max_lat_force_ref = v^2 * gradient(yaw_ref)*g;
        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%% Load the road gradient point cloud
            % [road_gradient, groundtruthptCloud] = F_depth2gradient(depthImage_read,groundtruth_read,colorImage_read,rawlidarImage_read,maxCameraDepth);
            % gt_width_from_center = interp1(x_ref,y_ref,groundtruthptCloud.Location(:,1));
            % show_idx = groundtruthptCloud.Location(:,2) < gt_width_from_center+lane_width/2-0.1 & groundtruthptCloud.Location(:,2) > gt_width_from_center-lane_width/2+0.1;
            % road_width_from_center = interp1(x_ref,y_ref,road_gradient(:,1));
            % gradient_idx = road_gradient(:,2) < road_width_from_center+lane_width/2 & road_gradient(:,2) > road_width_from_center-lane_width/2;% & road_gradient(:,1) < 14;
        
            % road_gradient = pointCloud(road_gradient(gradient_idx,:));
            % % road_gradient = pcdenoise(road_gradient,"Threshold",0.1,"NumNeighbors",3,"PreserveStructure",false);
            % minDistance = 0.4;
            % minPoints = 15;
            % [label, numClusters] = pcsegdist(road_gradient,minDistance,'NumClusterPoints',minPoints);%,'ParallelNeighborSearch',true);
            % road_gradient = road_gradient.Location(label>0,:); label = label(label>0);
            % groundtruthptCloud = select(groundtruthptCloud,show_idx);
        
            % figure;
            % subplot(3,1,1);
            % pcshow(groundtruthptCloud);
            % xlabel('\itX \rm[m]');
            % ylabel('\itY \rm[m]');
            % fontname(gcf,"Arial");
            % fontsize(gca,8,"points");
            % set(gcf,'color','w');
            % set(gca,'color','w');
            % set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
            % xlim([0 30])
            % view([0 90])
            % hold off;
        
            % figure;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
            % Define environment parameters
            % Define the obstacle as the mean position of the points in the gradient point cloud
            % obstacle = zeros(numClusters, 5);
            disp(['The number of clusters is: ', num2str(numClusters)]);
            % label_count = double(label);
            % for j = 1:numClusters
            %     label_count(label==j) = sum(label==j);
            %     cluster = road_gradient(label==j,:);
        
            %     % Calculate the center of grabity of the cluster
            %     weighted_sum = sum(cluster(:, 1:2) .* cluster(:, 3), 1);
            %     total_weight = sum(cluster(:, 3));
            %     mean_height = mean(total_weight);
            %     center_of_gravity = weighted_sum / total_weight;
            %     % center_of_gravity = weighted_sum / height(cluster);
        
            %     % Calculate mean and standard deviation for x and y axis
            %     % mean_x = mean(cluster(:, 1));
            %     % mean_y = mean(cluster(:, 2));
            %     std_x = std(cluster(:, 1));
            %     std_y = std(cluster(:, 2));
        
            %     obstacle(j,:) = [center_of_gravity,mean_height,std_x,std_y];
                
            %     % disp(['Cluster ', num2str(j), ' - Mean X: ', num2str(mean_x), ', Std X: ', num2str(std_x)]);
            %     % disp(['Cluster ', num2str(j), ' - Mean Y: ', num2str(mean_y), ', Std Y: ', num2str(std_y)]);
            % end
            std_x_obs = 2*obstacle_radius/3; % Standard deviation in x-axis
            std_y_obs = 2*obstacle_radius/3; % Standard deviation in y-axis
            % height_obs = 0.02; % Height of the obstacle
            obstacle = [x_obs, y_obs, height_obs, std_x_obs, std_y_obs];
        
            % obstacle = [obstacle; last_obs];
            % numClusters = height(obstacle);
            obstacle_calculation = ~isempty(obstacle);
            % obstacle = mean(road_gradient, 1)';
            % obstacle = obstacle(1:2);
            % disp(['The obstacle position is: ', num2str(obstacle')]);
        
            % Define the potential field parameters
            repulsive_gain = 100;
            center_gain = 0.1;
            delta_gain = 0.01;
            lateral_G_gain = 0.08;
            pdf_sigma = 0.3;
        
            % Initial conditions
            x0 = [0; 0; 0; v; 0; 0];
            % x0 = next_state;
            R = [cos(x0(3)), -sin(x0(3)); sin(x0(3)), cos(x0(3))];
            current_xy = R * [wb/2; 0];
            x0(1) = current_xy(1);
            % x0(5) = atan(0.5*tan(current_input));
            % x0(1) = ;
            % u0 = 0.0;
            % u0 = next_input- sign(next_input).*[0.00000001];
            % last_du = 0;
            last_du = current_du;
        
            % Define the maximum allowable lateral force
            max_avoidance_lat_force = 2;
            max_lat_force_ref(abs(max_lat_force_ref) < max_avoidance_lat_force) = max_avoidance_lat_force;
            max_lateral_force = abs(max_lat_force_ref);
        
            % Define the maximum allowable yaw rate
            max_yaw_rate = 0.06;
        
            % Define the maximum allowable lateral position
            max_lateral_position = lane_width/2 - tw/2 - 0.3;
        
            % Define the goal position
            goal = [v*dt*N; 0; 0];
        
            for k = 1:N+1
                yaw_ref_angle = yaw_ref(k);
                R_ref = [cos(yaw_ref_angle), -sin(yaw_ref_angle); sin(yaw_ref_angle), cos(yaw_ref_angle)];
                left_wheel_pos_ref = [x_ref(k); y_ref(k)] + R_ref * [wb/2; tw/2];
                right_wheel_pos_ref = [x_ref(k); y_ref(k)] + R_ref * [wb/2; -tw/2];
        
                yaw_angle = x(3,k);
                R = [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)];
                left_wheel_pos = x(1:2,k) + R * [wb/2; tw/2];
                right_wheel_pos = x(1:2,k) + R * [wb/2; -tw/2];
                if k <= N
                    % Internal model
                    % dx = F_KinematicBicycleModel(v, x(:,k), u(:,k), wb);
                    [dx, Fy] = F_DynamicBicycleModel(v, x(:,k), u(:,k), wb, dt, mass, Iz);
                    opti.subject_to(x(:,k+1) == x(:,k) + dt*dx);
                    if k < N
                        [~, Fy1] = F_DynamicBicycleModel(v, x(:,k+1), u(:,k+1), wb, dt, mass, Iz);
                        % opti.subject_to(-0.5 < (Fy1-Fy)/dt < 0.5);
                    end
                    if k > 1
                        opti.subject_to(u(:,k) == u(:,k-1) + du(:,k));
                        % opti.subject_to(-opt_params_max_yaw_jerk < du(:,k)-du(:,k-1) < opt_params_max_yaw_jerk);
                    else
                        opti.subject_to(u(:,k) == current_input + du(:,k));
                        % opti.subject_to(-opt_params_max_yaw_jerk < du(:,k)-current_du < opt_params_max_yaw_jerk);
                    end
                    % opti.subject_to(-max_yaw_rate <= x(3,k+1)-x(3,k) <= max_yaw_rate);
        
                    % Lateral force constraint
                    % lateral_force = v^2 * tan(u(:,k)) / wb;
                    % opti.subject_to(-max_lateral_force(k) <= lateral_force <= max_lateral_force(k));
                    opti.subject_to(-max_lateral_force(k) <= Fy <= max_lateral_force(k));
                end
                % opti.subject_to(-max_yaw_rate < x(6,k) < max_yaw_rate);
                % opti.subject_to(-opt_params_max_yaw_jerk < dx(6) < opt_params_max_yaw_jerk);
                % Boundary constraints
                ref = [x_ref(k); y_ref(k)];
                opti.subject_to(ref(1)-1 < x(1,k) < ref(1)+1);
                % opti.subject_to(ref(2)-max_lateral_position <= x(2,k) <= ref(2)+max_lateral_position);
                opti.subject_to(left_wheel_pos_ref(2)-max_lateral_position <= left_wheel_pos(2) <= left_wheel_pos_ref(2)+max_lateral_position);
                opti.subject_to(right_wheel_pos_ref(2)-max_lateral_position <= right_wheel_pos(2) <= right_wheel_pos_ref(2)+max_lateral_position);
            end
        
            % Define the cost function
            cost = 0;
            for k = 1:N+1
                ref = [x_ref(k); y_ref(k);yaw_ref(k)];
                % Calculate wheel positions considering yaw angle
                yaw_angle = x(3,k);
                R = [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)];
                left_wheel_pos = x(1:2,k) + R * [wb/2; tw/2];
                right_wheel_pos = x(1:2,k) + R * [wb/2; -tw/2];
        
                % Attractive potential
                if k <= N
                    [dx, Fy] = F_DynamicBicycleModel(v, x(:,k), u(:,k), wb, dt, mass, Iz);
                    cost = cost + delta_gain * (du(k)^2);
                    cost = cost + delta_gain * (x(6,k)^2);
        
                    % Lateral force constraint
                    cost = cost + lateral_G_gain * Fy^2;
        
                    % lateral_force = v^2 * tan(u(:,k)) / wb;
                    % cost = cost + lateral_G_gain * lateral_force^2;
        
                    % Calculate next step wheel positions
                    yaw_angle_next = x(3,k+1);
                    R_next = [cos(yaw_angle_next), -sin(yaw_angle_next); sin(yaw_angle_next), cos(yaw_angle_next)];
                    left_wheel_pos_next = x(1:2,k+1) + R_next * [wb/2; tw/2];
                    right_wheel_pos_next = x(1:2,k+1) + R_next * [wb/2; -tw/2];
        
                    % interpolate the wheel positions
                    left_wheel_itpl = [linspace(left_wheel_pos(1), left_wheel_pos_next(1), interp_steps)'; ...
                                    linspace(left_wheel_pos(2), left_wheel_pos_next(2), interp_steps)'];
                    right_wheel_itpl = [linspace(right_wheel_pos(1), right_wheel_pos_next(1), interp_steps)'; ...
                                        linspace(right_wheel_pos(2), right_wheel_pos_next(2), interp_steps)'];
                    left_wheel_itpl = left_wheel_itpl(:,1:end-1);
                    right_wheel_itpl = right_wheel_itpl(:,1:end-1);
        
                    x_interpolated = [linspace(x(1,k), x(1,k+1), interp_steps)'; linspace(x(2,k), x(2,k+1), interp_steps)'; linspace(x(3,k), x(3,k+1), interp_steps)'];
                    ref_interpolated = [linspace(ref(1), x_ref(k+1), interp_steps); linspace(ref(2), y_ref(k+1), interp_steps); linspace(ref(3), yaw_ref(k+1), interp_steps)];
                    x_interpolated = x_interpolated(:,1:end-1);
                    ref_interpolated = ref_interpolated(:,1:end-1);
        
                    for i = 1:width(left_wheel_itpl)
                        % Repulsive potential for each wheel position
                        if obstacle_calculation
        
                            for j = 1:numClusters
                                repulsive_left_x = repulsive_gain * obstacle(j,3) * F_pdf(left_wheel_itpl(1,i)', obstacle(j,1), (v)*obstacle(j,4), true); % Gaussian
                                repulsive_left_y = repulsive_gain * obstacle(j,3) * F_pdf(left_wheel_itpl(2,i)', obstacle(j,2), obstacle(j,5), false); % Gaussian
                                cost = cost + repulsive_left_x * repulsive_left_y;
        
                                repulsive_left_x = repulsive_gain * obstacle(j,3) * F_pdf(right_wheel_itpl(1,i)', obstacle(j,1), (v)*obstacle(j,4), true); % Gaussian
                                repulsive_left_y = repulsive_gain * obstacle(j,3) * F_pdf(right_wheel_itpl(2,i)', obstacle(j,2), obstacle(j,5), false); % Gaussian
                                cost = cost + repulsive_left_x * repulsive_left_y;
                            end
                        end
        
                    end
                end
                % Centering potential
                cost = cost + center_gain * sum((ref_interpolated(1:2,i) - x_interpolated(1:2,i)).^2);
                cost = cost + center_gain * sum((ref_interpolated(3,i) - x_interpolated(3,i)).^2);
        
                % Centering potential
                % ref = [x_ref(k); y_ref(k)];
                % cost = cost + center_gain * k^2 * sum((ref - x(1:2,k)).^2);
            end
            opti.minimize(cost);
        
            % Define the initial and terminal constraints
            % opti.subject_to(u(1:length(u0)) == u0); % Start at origin
            % opti.subject_to(du(1) == last_du); % Start at origin
            opti.subject_to(x(:,1) == x0); % Start at origin
            opti.subject_to(x(3,end) == yaw_ref(end)); % Start at origin
        
            % Set initial guess
            opti.set_initial(x, [x_ref; y_ref; zeros(1, N+1); v*ones(1, N+1); zeros(1, N+1); zeros(1, N+1)]);
            du_guess = zeros(1, N);
            du_guess(1,1) = 0;
            opti.set_initial(du, du_guess);
        
            % Solve the optimization problem
            opts = struct;
            opts.ipopt.print_level = 0;
            opts.print_time = true;
            opti.solver('ipopt', opts);
            tic
            sol = opti.solve();
            toc
        
            % Extract the solution
            x_sol = sol.value(x);
            u_sol = sol.value(du);
            wheel_sol = sol.value(u);
            % last_obs = obstacle(obstacle(:,1)<min(groundtruthptCloud.Location(:,1))+v*dt&obstacle(:,1)>0,:);
            % last_obs(:,1) = last_obs(:,1) - v*dt;
        
            distances = [];
            for kk = 1:length(x_sol)-1
                if x_sol(1,kk) < x_obs && x_sol(1,kk+1) > x_obs
                    % Calculate the distance to the obstacle
                    slope = (x_sol(2,kk+1) - x_sol(2,kk)) / (x_sol(1,kk+1) - x_sol(1,kk));
                    bias = x_sol(2,kk) - slope * x_sol(1,kk);
                    distance_to_obs = abs(slope*x_obs + (-1)*y_obs + bias) / sqrt(slope^2 + (-1)^2);
                    distances = [distances, distance_to_obs];
                end
            end
            distance_to_obstacle_list = [distance_to_obstacle_list, min(distances)];
        
            % Next state
            next_state = x_sol(:,2);
        
            % Extract the optimal input
            current_input = wheel_sol(1);
            current_du = u_sol(1);
            disp(['The next input is: ', num2str(current_input)]);
        
            % Max lateral force
            beta = atan(0.5*tan(wheel_sol));
            vx = x_sol(4,1:end-1);
            vy = x_sol(5,1:end-1); % 車両横方向速度
            omega = x_sol(6,1:end-1); % ヨーレート
            Fyf = -100e3 * ((vy + L/2 * omega)./vx - wheel_sol); % 前輪横力
            % Fyf = -300e3 * ((beta + (L/2) * omega)./vx - wheel_sol); % 前輪横力
            Fyr = -100e3 * (vy - L/2 * omega)./vx;             % 後輪横力
            % Fyr = -300e3 * (beta-((L/2) * omega)./vx);             % 後輪横力
            dot_beta = (L/2 * Fyf - L/2 * Fyr) / Iz;  % 横滑り角の変化率
            % ay = (vx / mass) .* (dot_beta + (L/2 ./ vx).* Fyf - (L/2 ./ vx).* Fyr);  % 横加速度
            ay = (Fyf .* cos(wheel_sol) / mass + Fyr / mass - vx .* omega);  % 横加速度
            max_lateral_force_list = [max_lateral_force_list, max(abs(ay))];
        
            % lateral_force = abs(v^2 * tan(wheel_sol) / L)/g;
            % max_lateral_G = max(abs(v^2 * tan(wheel_sol) / L)/g);
            max_lateral_G = max(abs(ay)/g);
            % max_lateral_G = max((abs(Fyf.*sin(wheel_sol)+Fyr)/mass)/g);
            % idx = find(lateral_force > max_lateral_G-0.00001);
            idx = find(abs(ay) > max_avoidance_lat_force-0.00001);
            disp(['The max lateral G is: ', num2str(max_lateral_G)]);
        
            % subplot(2,1,1);
            if x_obs < max(x_sol(1,:))
                if min(distances)-obstacle_radius > tw/2+0.075 || min(distances)+obstacle_radius < tw/2-0.075
                    clr = 'r';
                else
                    clr = 'b';
                    if ~over_flag
                        over_flag = true;
                        over_x = x_obs;
                    end
                end
            else
                clr = 'r';
            end
            % viscircles([x_obs, y_obs], obstacle_radius, 'EdgeColor', clr);
            scatter(x_obs, y_obs, 40, clr, 'filled','MarkerFaceAlpha',0.1);
            grid on; axis equal;
            xlim([0 25])
            ylim([-lane_width/2 lane_width/2])
            % view([0 90])
            xlabel('\itX \rm[m]');
            ylabel('\itY \rm[m]');
            fontname(gcf,"Times New Roman");
            fontsize(gca,10,"points");
            % set(gcf,'color','w');
            % set(gca,'color','w');
            % set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
            hold on;
            drawnow;
        
            % Plot the reference trajectory
            % figure;
            if plot_sol
                subplot(2,1,2);
                % Calculate the total potential field
                [X, Y] = meshgrid(0:0.1:25, -lane_width/2:0.1:lane_width/2);
                total_potential = zeros(size(X));
            
                for i = 1:size(X, 1)
                    for j = 1:size(X, 2)
                        pos = [X(i, j); Y(i, j)];
                        % Calculate the repulsive potential
                        if obstacle_calculation
                            % dist_to_obstacle = sum((obstacle - pos').^2, 2);
                            % point_repulsive = repulsive_gain  * obstacle_gain * sum(1./(((dist_to_obstacle/3).^2) + 1e-3));
                            % total_potential(i, j) = total_potential(i, j) + point_repulsive;
                            for k = 1:numClusters
                                % dist_to_obstacle = sum((road_gradient(:,1:2) - repmat(pos', [height(road_gradient), 1])).^2, 2);
                                % repulsive_potential = repulsive_gain * obstacle_gain * k^2 * road_gradient(:,3)' * (1 ./ (dist_to_obstacle + 1e-3)); % 1/d^2
                                % repulsive_potential = repulsive_gain * obstacle_gain * road_gradient(:,3)' * F_pdf(dist_to_obstacle, 0, pdf_sigma, false); % Gaussian
                                % repulsive_potential_x = repulsive_gain *  (road_gradient(label==k,3)'./(label_count(label==k).^2)') * F_pdf(repmat(pos(1), [height(road_gradient(label==k,3)), 1]), road_gradient(label==k,1), v*pdf_sigma, true); % Gaussian
                                % repulsive_potential_y = repulsive_gain * (road_gradient(label==k,3)'./(label_count(label==k).^2)') * F_pdf(repmat(pos(2), [height(road_gradient(label==k,3)), 1]), road_gradient(label==k,2), pdf_sigma, false); % Gaussian
                                repulsive_potential_x = repulsive_gain * obstacle(k,3) * F_pdf(pos(1), obstacle(k,1), (v)*obstacle(k,4), true);
                                repulsive_potential_y = repulsive_gain * obstacle(k,3) * F_pdf(pos(2), obstacle(k,2), obstacle(k,5), false);
            
                                total_potential(i, j) = total_potential(i, j) + repulsive_potential_x * repulsive_potential_y;
                            end
                        end
            
                        centering_potential = center_gain * sum((pos - [X(i, j); 0]).^2);
                        total_potential(i, j) = total_potential(i, j) + centering_potential;
                    end
                end
            
                % Ground truth point cloud
                % lower_gtpoint = groundtruthptCloud.Location;
                % lower_gtpoint(:,3) = lower_gtpoint(:,3) - 0.05;
                % lower_gtpoint = pointCloud(lower_gtpoint,"Color",groundtruthptCloud.Color);
                % pcshow(lower_gtpoint); hold on; % Plot the ground truth
            
                % Plot the total potential field
                [c,h] = contourf(X, Y, total_potential, 20);
                colormap(parula);
                clim([0 10])
                h.FaceAlpha = 0.5; hold on;
                % colorbar; hold on;
            
                plot(x_ref, y_ref, 'g--', 'LineWidth', 2); hold on;
            
                % Calculate the lane boundaries considering yaw angle derived from reference position
                yaw_ref = atan2(diff(y_ref), diff(x_ref));
                yaw_ref = [yaw_ref, yaw_ref(end)]; % Extend the last yaw angle to match the size
            
                boundary_l = zeros(2, N+1);
                boundary_r = zeros(2, N+1);
                for k = 1:N+1
                    yaw_angle = yaw_ref(k);
                    boundary_l(:,k) = [x_ref(k); y_ref(k)] + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [0; lane_width/2];
                    boundary_r(:,k) = [x_ref(k); y_ref(k)] + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [0; -lane_width/2];
                end
            
                % Plot the lane boundaries
                plot(boundary_l(1,:), boundary_l(2,:), 'Color', '#FFA500', 'LineWidth', 2);
                plot(boundary_r(1,:), boundary_r(2,:), 'Color', '#FFA500', 'LineWidth', 2);
            
                % Plot the solution path
                z = 0.0 * ones(1, N+1);
                plot(x_sol(1,:), x_sol(2,:), 'r-o');
            
                % Calculate and plot wheel positions considering yaw angle
                rl_wheel_pos = zeros(2, N+1);
                rr_wheel_pos = zeros(2, N+1);
                fl_wheel_pos = zeros(2, N+1);
                fr_wheel_pos = zeros(2, N+1);
                for k = 1:N+1
                    yaw_angle = x_sol(3,k);
                    rl_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [-wb/2; tw/2];
                    rr_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [-wb/2; -tw/2];
                    fl_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [wb/2; tw/2];
                    fr_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [wb/2; -tw/2];
                end
            
                plot(rl_wheel_pos(1,:), rl_wheel_pos(2,:), '-blue*');
                plot(rr_wheel_pos(1,:), rr_wheel_pos(2,:), '-blue*');
                plot(fl_wheel_pos(1,:), fl_wheel_pos(2,:), '-cyan*');
                plot(fr_wheel_pos(1,:), fr_wheel_pos(2,:), '-cyan*');
            
                % Point of max lateral force
                scatter(x_sol(1, idx), x_sol(2, idx), 60, 'magenta', 'filled');
            
                % Plot the obstacle
                % viscircles(obstacle', obstacle_radius, 'EdgeColor', 'r');
                % scatter3(obstacle(1), obstacle(2), 0.03, 60, 'r', 'filled');
            
                grid on; axis equal;
                xlim([0, 25]); % X-axis limits
                ylim([-lane_width/2, lane_width/2]); % Lane width
                % view([0 90]);
            
                % Labels and title
                % xlabel('\itX \rm[m]');
                % ylabel('\itY \rm[m]');
                % set(gcf,'color','w');
                % set(gca,'color','w');
                % set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
                hold off;
                xlabel('\itX \rm[m]');
                ylabel('\itY \rm[m]');
                fontname(gcf,"Times New Roman");
                fontsize(gca,10,"points");
            end
        
            drawnow;
            clear opti
            if animation
                frame = getframe(gcf);
                writeVideo(video,frame);
            end
        end
    end
    if animation
        close(video);
    end
    xline(over_x+obstacle_radius/2,"-k","Border"+newline+round(over_x,2)+" m",'LabelHorizontalAlignment','right','LabelVerticalAlignment','bottom','LabelOrientation','aligned')
    fontname(gcf,"Times New Roman");
    fontsize(gca,10,"points");
end