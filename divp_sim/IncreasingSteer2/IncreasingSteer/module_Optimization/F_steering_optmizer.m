function steer_cmd = F_steering_optmizer(opt_params,unevenness_points,states,x_ref,y_ref,yaw_ref,v,steer)
    addpath('./casadi-3.6.7-windows64-matlab2018b')
    % opti = casadi.Opti();
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  NLMPC code block
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % import casadi.*
    
    % Clear and close all
    % clear;
    % close all;
    
    % Define the vehicle parameters
    tw = opt_params.tw; % Track width
    wb = opt_params.wb; % Wheelbase
    % v = 50/3.6; % Constant velocity
    
    lane_width = 3.5; % Width of the lane
    g = 9.8;
    
    % Define the optimization variables
    opti = casadi.Opti();
    N = opt_params.N; % Number of steps
    interp_steps = opt_params.interp_steps;
    x = opti.variable(3, N+1); % State variables (x, y, theta)
    u = opti.variable(1, N);   % Control variables
    du = opti.variable(1, N); % Change in control variables
    
    % Define the dynamics
    dt = opt_params.dt;
    
    % Define reference trajectory
    % x_ref = 0:dt*v:dt*v*N;
    % y_ref = zeros(size(x_ref));
    % y_ref = -(exp(x_ref*0.02) - 1);
    % yaw_ref = atan2(gradient(y_ref), gradient(x_ref));
    max_lat_force_ref = v^2 * gradient(yaw_ref)*g;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Load the road gradient point cloud
    % unevenness_points = pcdenoise(unevenness_points,"Threshold",0.1,"NumNeighbors",3,"PreserveStructure",false);
    minDistance = 0.05;
    minPoints = 50;
    [label, numClusters] = pcsegdist(unevenness_points,minDistance,'NumClusterPoints',minPoints);%,'ParallelNeighborSearch',true);
    unevenness_points = unevenness_points.Location(label>0,:); label = label(label>0);
    
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
    subplot(3,1,2);
    pcshow(unevenness_points)
    xlim([0 30])
    ylim([-3.5/2 3.5/2])
    view([0 90])
    xlabel('\itX \rm[m]');
    ylabel('\itY \rm[m]');
    fontname(gcf,"Arial");
    fontsize(gca,8,"points");
    set(gcf,'color','w');
    set(gca,'color','w');
    set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
    hold off;
    drawnow;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Define environment parameters
    % Define the obstacle as the mean position of the points in the gradient point cloud
    obstacle = zeros(numClusters, 5);
    disp(['The number of clusters is: ', num2str(numClusters)]);
    label_count = double(label);
    for j = 1:numClusters
        label_count(label==j) = sum(label==j);
        cluster = unevenness_points(label==j,:);
    
        % Calculate the center of grabity of the cluster
        weighted_sum = sum(cluster(:, 1:2) .* cluster(:, 3), 1);
        total_weight = sum(cluster(:, 3));
        mean_height = mean(total_weight);
        center_of_gravity = weighted_sum / total_weight;
    
        std_x = std(cluster(:, 1));
        std_y = std(cluster(:, 2));
    
        obstacle(j,:) = [center_of_gravity,mean_height,std_x,std_y];
    end
    obstacle_calculation = ~isempty(obstacle);
    % obstacle = mean(unevenness_points, 1)';
    % obstacle = obstacle(1:2);
    % disp(['The obstacle position is: ', num2str(obstacle')]);
    
    % Define the potential field parameters
    repulsive_gain = opt_params.repulsive_gain;
    center_gain = opt_params.center_gain;
    delta_gain = opt_params.delta_gain;
    lateral_G_gain = opt_params.lateral_G_gain;
    
    % Initial conditions
    % x0 = next_state;
    x0 = [0; 0; states(3)]; % x,y,theta
    
    % Define the maximum allowable lateral force
    max_avoidance_lat_force = 0.2 * g;
    max_lat_force_ref(abs(max_lat_force_ref) < max_avoidance_lat_force) = max_avoidance_lat_force;
    max_lateral_force = abs(max_lat_force_ref);
    
    % Define the maximum allowable yaw rate
    max_yaw_rate = opt_params.max_yaw_rate;
    
    % Define the maximum allowable lateral position
    max_lateral_position = lane_width/2 - tw/2 - 0.35;
    
    for k = 1:N+1
        yaw_ref_angle = yaw_ref(k);
        R_ref = [cos(yaw_ref_angle), -sin(yaw_ref_angle); sin(yaw_ref_angle), cos(yaw_ref_angle)];
        left_wheel_pos_ref = [x_ref(k); y_ref(k)] + R_ref * [wb; tw/2];
        right_wheel_pos_ref = [x_ref(k); y_ref(k)] + R_ref * [wb; -tw/2];
    
        yaw_angle = x(3,k);
        R = [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)];
        left_wheel_pos = x(1:2,k) + R * [wb; tw/2];
        right_wheel_pos = x(1:2,k) + R * [wb; -tw/2];
        if k <= N
            % Internal model
            dx = F_KinematicBicycleModel(v, x(:,k), u(:,k), wb);
            opti.subject_to(x(:,k+1) == x(:,k) + dt*dx);
            if k > 1
                opti.subject_to(u(:,k) == u(:,k-1) + du(:,k));
            else
                opti.subject_to(u(:,k) == steer + du(:,k));
            end
            opti.subject_to(-max_yaw_rate <= du(k) <= max_yaw_rate);
    
            % Lateral force constraint
            lateral_force = v^2 * tan(u(:,k)) / wb;
            opti.subject_to(-max_lateral_force(k) <= lateral_force <= max_lateral_force(k));
        end
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
        ref = [x_ref(k); y_ref(k)];
        % Calculate wheel positions considering yaw angle
        yaw_angle = x(3,k);
        R = [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)];
        left_wheel_pos = x(1:2,k) + R * [wb; tw/2];
        right_wheel_pos = x(1:2,k) + R * [wb; -tw/2];
    
        % Attractive potential
        if k <= N
            cost = cost + delta_gain * (du(k)^2);
    
            % Lateral force constraint
            lateral_force = v^2 * tan(u(:,k)) / wb;
            cost = cost + lateral_G_gain * lateral_force^2;
    
            % Calculate next step wheel positions
            yaw_angle_next = x(3,k+1);
            R_next = [cos(yaw_angle_next), -sin(yaw_angle_next); sin(yaw_angle_next), cos(yaw_angle_next)];
            left_wheel_pos_next = x(1:2,k+1) + R_next * [wb; tw/2];
            right_wheel_pos_next = x(1:2,k+1) + R_next * [wb; -tw/2];
    
            % interpolate the wheel positions
            left_wheel_itpl = [linspace(left_wheel_pos(1), left_wheel_pos_next(1), interp_steps)'; ...
                               linspace(left_wheel_pos(2), left_wheel_pos_next(2), interp_steps)'];
            right_wheel_itpl = [linspace(right_wheel_pos(1), right_wheel_pos_next(1), interp_steps)'; ...
                                linspace(right_wheel_pos(2), right_wheel_pos_next(2), interp_steps)'];
            left_wheel_itpl = left_wheel_itpl(:,1:end-1);
            right_wheel_itpl = right_wheel_itpl(:,1:end-1);
    
            x_interpolated = [linspace(x(1,k), x(1,k+1), interp_steps)'; linspace(x(2,k), x(2,k+1), interp_steps)'];
            ref_interpolated = [linspace(ref(1), x_ref(k+1), interp_steps); linspace(ref(2), y_ref(k+1), interp_steps)];
            x_interpolated = x_interpolated(:,1:end-1);
            ref_interpolated = ref_interpolated(:,1:end-1);
    
            for i = 1:width(left_wheel_itpl)
                % Repulsive potential for each wheel position
                if obstacle_calculation
    
                    for j = 1:numClusters
                        repulsive_left_x = repulsive_gain * obstacle(j,3) * F_pdf(left_wheel_itpl(1,i)', obstacle(j,1), v*obstacle(j,4), true); % Gaussian
                        repulsive_left_y = repulsive_gain * obstacle(j,3) * F_pdf(left_wheel_itpl(2,i)', obstacle(j,2), obstacle(j,5), false); % Gaussian
                        cost = cost + repulsive_left_x * repulsive_left_y;
    
                        repulsive_left_x = repulsive_gain * obstacle(j,3) * F_pdf(right_wheel_itpl(1,i)', obstacle(j,1), v*obstacle(j,4), true); % Gaussian
                        repulsive_left_y = repulsive_gain * obstacle(j,3) * F_pdf(right_wheel_itpl(2,i)', obstacle(j,2), obstacle(j,5), false); % Gaussian
                        cost = cost + repulsive_left_x * repulsive_left_y;
                    end
                end
    
                % Centering potential
                cost = cost + center_gain * sum((ref_interpolated(:,i) - x_interpolated(1:2,i)).^2);
            end
        end
    
        % Centering potential
        % ref = [x_ref(k); y_ref(k)];
        % cost = cost + center_gain * k^2 * sum((ref - x(1:2,k)).^2);
    end
    opti.minimize(cost);
    
    % Define the initial and terminal constraints
    opti.subject_to(x(:,1) == x0); % Start at origin
    
    % Set initial guess
    opti.set_initial(x, [x_ref; y_ref; zeros(1, N+1)]);
    du_guess = zeros(1, N);
    opti.set_initial(du, du_guess);
    
    % Solve the optimization problem
    opts = struct;
    opts.ipopt.print_level = 0;
    opts.print_time = true;
    opti.solver('ipopt', opts);
    sol = opti.solve();
    
    % Extract the solution
    x_sol = sol.value(x);
    u_sol = sol.value(du);
    wheel_sol = sol.value(u);
    
    % Next state
    % next_state = x_sol(:,2);
    
    % Extract the optimal input
    steer_cmd = wheel_sol(1);
    current_du = u_sol(1);
    
    % Max lateral force
    lateral_force = abs(v^2 * tan(wheel_sol) / wb)/g;
    max_lateral_G = max(abs(v^2 * tan(wheel_sol) / wb)/g);
    idx = find(lateral_force > max_lateral_G-0.00001);
    disp(['The max lateral G is: ', num2str(max_lateral_G)]);
    
    % Plot the reference trajectory
    % figure;
    subplot(3,1,3);
    % Calculate the total potential field
    [X, Y] = meshgrid(0:0.1:30, -lane_width/2:0.1:lane_width/2);
    total_potential = zeros(size(X));
    
    for i = 1:size(X, 1)
        for j = 1:size(X, 2)
            pos = [X(i, j); Y(i, j)];
            % Calculate the repulsive potential
            if obstacle_calculation
                for k = 1:numClusters
                    repulsive_potential_x = repulsive_gain * obstacle(k,3) * F_pdf(pos(1), obstacle(k,1), v*obstacle(k,4), true);
                    repulsive_potential_y = repulsive_gain * obstacle(k,3) * F_pdf(pos(2), obstacle(k,2), obstacle(k,5), false);
    
                    total_potential(i, j) = total_potential(i, j) + repulsive_potential_x * repulsive_potential_y;
                end
            end
    
            centering_potential = center_gain * sum((pos - [X(i, j); 0]).^2);
            total_potential(i, j) = total_potential(i, j) + centering_potential;
        end
    end
    
    % Ground truth point cloud
    lower_gtpoint = groundtruthptCloud.Location;
    lower_gtpoint(:,3) = lower_gtpoint(:,3) - 0.05;
    lower_gtpoint = pointCloud(lower_gtpoint,"Color",groundtruthptCloud.Color);
    pcshow(lower_gtpoint); hold on; % Plot the ground truth
    
    % Plot the total potential field
    [c,h] = contourf(X, Y, total_potential, 20);
    colormap(parula);
    h.FaceAlpha = 0.1;
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
    plot3(x_sol(1,:), x_sol(2,:), z, 'r-o');
    
    % Calculate and plot wheel positions considering yaw angle
    rl_wheel_pos = zeros(2, N+1);
    rr_wheel_pos = zeros(2, N+1);
    fl_wheel_pos = zeros(2, N+1);
    fr_wheel_pos = zeros(2, N+1);
    for k = 1:N+1
        yaw_angle = x_sol(3,k);
        rl_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [0; tw/2];
        rr_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [0; -tw/2];
        fl_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [wb; tw/2];
        fr_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [wb; -tw/2];
    end
    
    plot3(rl_wheel_pos(1,:), rl_wheel_pos(2,:), z, '-blue*');
    plot3(rr_wheel_pos(1,:), rr_wheel_pos(2,:), z, '-blue*');
    plot3(fl_wheel_pos(1,:), fl_wheel_pos(2,:), z, '-cyan*');
    plot3(fr_wheel_pos(1,:), fr_wheel_pos(2,:), z, '-cyan*');
    
    % Point of max lateral force
    scatter3(x_sol(1, idx), x_sol(2, idx), 0, 60, 'magenta', 'filled');
    
    grid on; axis equal;
    xlim([0, 30]); % X-axis limits
    ylim([-lane_width/2, lane_width/2]); % Lane width
    view([0 90]);
    
    % Labels and title
    xlabel('\itX \rm[m]');
    ylabel('\itY \rm[m]');
    set(gcf,'color','w');
    set(gca,'color','w');
    set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
    hold off;
    
    drawnow;
    clear opti
end