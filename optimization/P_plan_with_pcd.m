%% define data folders
% dataset="val_selection_cropped/";
% dataset="val_selection_cropped_fixed_supervision/";
% dataset="val_selection_cropped_labeled_supervision/";
dataset=uigetdir("../sensing/rosbag_reader/ouster-dual/val_selection/", "DATASET folder to Open") + "\";

% figname="unpretrained_unfixed_supervision";
% figname="pretrained_fixed_supervision";
figname="pretrained_fixed_labeled_supervision";
% figname="conventional_model";
results=uigetdir("../sensing/rosbag_reader/ouster-dual/results/","RESULTS folder to Open") + "\results\";

list_predicted_imgs = dir(results+"*.png");
list_rawlidar_imgs  = dir(dataset+"velodyne_raw/*.png");
list_color_imgs     = dir(dataset+"image/*.png");
groundtruth_imgs    = dir(dataset+"groundtruth_depth/*.png");

%% read datas
close all;

file_name = "depth_image_008850.png";
file_num = 0;
flag = true;
while flag
    file_num = file_num+1;
    name = list_predicted_imgs(file_num).name;
    if name==file_name
        flag=false;
    end
    if file_num == length(list_predicted_imgs)-1
        flag=false;
    end
end
file_num=50;

rawlidarImage_read  = imread(dataset+"velodyne_raw/"+list_rawlidar_imgs(file_num).name);
predictedImage_read = imread(results+list_predicted_imgs(file_num).name);
colorImage_read     = imread(dataset+"image/"+list_color_imgs(file_num).name);
groundtruth_read    = imread(dataset+"groundtruth_depth/"+groundtruth_imgs(file_num).name);

images = {rawlidarImage_read;
            predictedImage_read;
            colorImage_read;
            groundtruth_read};

% if height(predictedImage_read) > 264
%     for i = 1:4
%         image = images{i};
%         image = image(1:264,:,:);
%         images{i} = image;
%     end
% end

rawlidarImage_read = images{1};
predictedImage_read = images{2};
colorImage_read = images{3};
groundtruth_read = images{4};

depthImage_read = images{2};

depthImage_check  = double(depthImage_read);
groundtruth_check = double(groundtruth_read);
depthImage_check(depthImage_check==0)   = nan;
groundtruth_check(groundtruth_check==0) = nan;
maxCameraDepth   = 30;
rmse_px = rmse(depthImage_check.*maxCameraDepth./65535,groundtruth_check.*maxCameraDepth./65535,"omitnan");
rmse_px = rmmissing(rmse_px);
RMSE_on_depthmap = sum(rmse_px,'all')/numel(rmse_px)

[gradient, groundtruthptCloud] = F_depth2gradient(depthImage_read,groundtruth_read,colorImage_read,rawlidarImage_read,maxCameraDepth);
show_idx = groundtruthptCloud.Location(:,2) < 3.5/2 & groundtruthptCloud.Location(:,2) > -3.5/2;
gradient_idx = gradient(:,2) < 3.5/2 & gradient(:,2) > -3.5/2;% & gradient(:,1) < 14;

gradient = gradient(gradient_idx,:);
groundtruthptCloud = select(groundtruthptCloud,show_idx);

figure;
subplot(3,1,1);
pcshow(groundtruthptCloud);
xlabel('\itX \rm[m]');
ylabel('\itY \rm[m]');
fontname(gcf,"Arial");
fontsize(gca,8,"points");
set(gcf,'color','w');
set(gca,'color','w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
xlim([0 30])
view([0 90])

% figure;
subplot(3,1,2);
pcshow(gradient)
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


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  NLMPC code block
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pause(1)
addpath('./casadi-3.6.7-windows64-matlab2018b')
import casadi.*

% Clear and close all
% clear;
% close all;

% Define the vehicle parameters
tw = 1.485; % Track width
wb = 2.86; % Wheelbase
v = 50/3.6; % Constant velocity
L = 3; % Wheelbase

% Define environment parameters
% Define the obstacle as the mean position of the points in the gradient point cloud
obstacle = mean(gradient, 1)';
obstacle = obstacle(1:2);
disp(['The obstacle position is: ', num2str(obstacle')]);

% obstacle = [13.6512; 0.482284];
lane_width = 3.5; % Width of the lane
g = 9.8;

% Define the potential field parameters
obstacle_radius = 0.25;
repulsive_gain = 40;
center_gain = 10;
delta_gain = 0.5;
lateral_G_gain = 6;

% Initial conditions
x0 = [0; 0; 0];
u0 = 0.0;
last_du = 0;

% Define the maximum allowable lateral force
max_lateral_force = 0.2 * g;

% Define the maximum allowable lateral position
max_lateral_position = 0.75;

% Define the optimization variables
opti = Opti();
N = 13; % Number of steps
x = opti.variable(3, N+1); % State variables (x, y, theta)
u = opti.variable(1, N);   % Control variables
du = opti.variable(1, N); % Change in control variables

% Define the dynamics
dt = 0.15;

% Define reference trajectory
x_ref = 0:dt*v:dt*v*N;
y_ref = zeros(size(x_ref));

% Define the goal position
goal = [v*dt*N; 0; 0];

for k = 1:N+1
    if k <= N
        % Internal model
        dx = F_KinematicBicycleModel(v, x(:,k), u(:,k), L);
        opti.subject_to(x(:,k+1) == x(:,k) + dt*dx);
        if k > 1
            opti.subject_to(u(:,k) == u(:,k-1) + du(:,k));
        end
        opti.subject_to(-pi/10 < du(k) < pi/10);

        % Lateral force constraint
        lateral_force = v^2 * tan(u(:,k)) / L;
        opti.subject_to(-max_lateral_force <= lateral_force <= max_lateral_force);
    end
    % Boundary constraints
    ref = [x_ref(k); y_ref(k)];
    opti.subject_to(ref(1)-1 < x(1,k) < ref(1)+1);
    opti.subject_to(ref(2)-max_lateral_position < x(2,k) < ref(2)+max_lateral_position);
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
    
    % dist_to_obstacle_left_wheel = norm((left_wheel_pos + left_wheel_pos) / 2 - obstacle);
    % cost = cost + repulsive_gain * 1/(dist_to_obstacle_left_wheel^2 + 1e-3);

    % % Repulsive potential for right wheel
    % dist_to_obstacle_right_wheel = norm((right_wheel_pos + right_wheel_pos) / 2 - obstacle);
    % cost = cost + repulsive_gain * 1/(dist_to_obstacle_right_wheel^2 + 1e-3);

    % Attractive potential
    if k <= N
        cost = cost + delta_gain * (du(k)^2);

        % Lateral force constraint
        lateral_force = v^2 * tan(u(:,k)) / L;
        cost = cost + lateral_G_gain * lateral_force^2;

        % Calculate next step wheel positions
        yaw_angle_next = x(3,k+1);
        R_next = [cos(yaw_angle_next), -sin(yaw_angle_next); sin(yaw_angle_next), cos(yaw_angle_next)];
        left_wheel_pos_next = x(1:2,k+1) + R_next * [wb; tw/2];
        right_wheel_pos_next = x(1:2,k+1) + R_next * [wb; -tw/2];

        % interpolate the wheel positions
        left_wheel_itpl = [linspace(left_wheel_pos(1), left_wheel_pos_next(1), 4)'; ...
                           linspace(left_wheel_pos(2), left_wheel_pos_next(2), 4)'];
        right_wheel_itpl = [linspace(right_wheel_pos(1), right_wheel_pos_next(1), 4)'; ...
                            linspace(right_wheel_pos(2), right_wheel_pos_next(2), 4)'];
        x_interpolated = [linspace(x(1,k), x(1,k+1), 4)'; linspace(x(2,k), x(2,k+1), 4)'];
        ref_interpolated = [linspace(ref(1), x_ref(k+1), 4); linspace(ref(2), y_ref(k+1), 4)];
        for i = 1:length(left_wheel_itpl)
            dist_to_obstacle_left_wheel_itpl = norm(left_wheel_itpl(:,i) / 2 - obstacle);
            cost = cost + repulsive_gain * 1/(dist_to_obstacle_left_wheel_itpl^2 + 1e-3);
            dist_to_obstacle_right_wheel_itpl = norm(right_wheel_itpl(:,i) / 2 - obstacle);
            cost = cost + repulsive_gain * 1/(dist_to_obstacle_right_wheel_itpl^2 + 1e-3);

            % Centering potential
            cost = cost + center_gain * k * sum((ref_interpolated(:,i) - x_interpolated(1:2,i)).^2);
        end
        
        % dist_to_obstacle_left_wheel_next = norm((left_wheel_pos_next+left_wheel_pos)./2 - obstacle);
        % cost = cost + repulsive_gain * 1/(dist_to_obstacle_left_wheel_next^2 + 1e-3);
    
        % Repulsive potential for right wheel
        % dist_to_obstacle_right_wheel_next = norm((right_wheel_pos_next+right_wheel_pos)./2 - obstacle);
        % cost = cost + repulsive_gain * 1/(dist_to_obstacle_right_wheel_next^2 + 1e-3);
    end

    % Centering potential
    % ref = [x_ref(k); y_ref(k)];
    % cost = cost + center_gain * k^2 * sum((ref - x(1:2,k)).^2);
end
opti.minimize(cost);

% Define the initial and terminal constraints
opti.subject_to(u(1) == u0); % Start at origin
opti.subject_to(du(1) == last_du); % Start at origin
opti.subject_to(x(:,1) == x0); % Start at origin

% Set initial guess
opti.set_initial(x, [x_ref; y_ref; zeros(1, N+1)]);
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

% Extract the optimal input
next_input = wheel_sol(2);
disp(['The next input is: ', num2str(next_input)]);

% Max lateral force
lateral_force = abs(v^2 * tan(wheel_sol) / L)/g;
max_lateral_G = max(abs(v^2 * tan(wheel_sol) / L)/g);
idx = find(lateral_force > max_lateral_G-0.00001);
disp(['The max lateral G is: ', num2str(max_lateral_G)]);

% Plot the reference trajectory
% figure;
subplot(3,1,3);
pcshow(groundtruthptCloud); hold on; % Plot the ground truth
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
z = 0.01 * ones(1, N+1);
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
scatter3(x_sol(1, idx), x_sol(2, idx), 0.05, 60, 'magenta', 'filled');

% Plot the obstacle
% viscircles(obstacle', obstacle_radius, 'EdgeColor', 'r');
scatter3(obstacle(1), obstacle(2), 0.05, 60, 'r', 'filled');

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
