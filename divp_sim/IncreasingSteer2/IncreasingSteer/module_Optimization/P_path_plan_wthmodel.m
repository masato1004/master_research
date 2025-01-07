addpath('./casadi-3.6.7-windows64-matlab2018b')
import casadi.*

% clear;
close all;

% Define the vehicle parameters
tw = 1.485; % Track width
wb = 2.86; % Wheelbase
v = 50/3.6; % Constant velocity
L = 3; % Wheelbase

% Define environment parameters
obstacle = [12;0.75];
lane_width = 3.5; % Width of the lane
g = 9.8;

% Define the potential field parameters
obstacle_radius = 0.25;
repulsive_gain = 40;
% attractive_gain = 1e-3;
center_gain = 2;
delta_gain = 0.5;
lateral_G_gain = 6;
% y_ref = 0;

% Initial conditions
x0 = [0; -0.; -0.0];
u0 = 0.0;
last_du = 0;

% Define the maximum allowable lateral force
max_lateral_force = 0.2*g;

% Define the maximum allowable lateral position
max_lateral_position = 0.75;

% Define the optimization variables
opti = Opti();
N = 22; % Number of steps
x = opti.variable(3, N+1); % State variables (x, y, theta)
u = opti.variable(1, N);   % Control variables
du = opti.variable(1, N); % Change in control variables

% Define the dynamics
dt = 0.1;

% Define reference trajectory
x_ref = 0:dt*v:dt*v*N;
y_ref = -(x_ref*0.06).^2;
y_ref = zeros(size(x_ref));

% Define the goal position
goal = [v*dt*N; 0; 0];

for k = 1:N+1
    if k <= N
        % internal model
        dx = F_KinematicBicycleModel(v, x(:,k), u(:,k), L);
        opti.subject_to(x(:,k+1) == x(:,k) + dt*dx);
        if k > 1
            opti.subject_to(u(:,k) == u(:,k-1) + du(:,k));
        end
        opti.subject_to(-pi/10 < du(k) < pi/10);

        % lateral force constraint
        lateral_force = v^2 * tan(u(:,k)) / L;
        opti.subject_to(-max_lateral_force <= lateral_force <= max_lateral_force);
    end
    % boundary constraints
    ref = [x_ref(k);y_ref(k)];
    opti.subject_to(ref(1)-1 < x(1,k) < ref(1)+1);
    opti.subject_to(ref(2)-max_lateral_position < x(2,k) < ref(2)+max_lateral_position);
    % opti.subject_to(-pi/12 < x(3,k) < pi/12);
    
end

% Define the cost function
cost = 0;
for k = 1:N+1
    % Calculate wheel positions considering yaw angle
    yaw_angle = x(3,k);
    left_wheel_pos = x(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [wb; tw/2];
    right_wheel_pos= x(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [wb; -tw/2];
    
    % left_wheel_pos = x(1:2,k) + [0; tw/2];
    % right_wheel_pos = x(1:2,k) - [0; tw/2];

    % Attractive potential
    if k <= N
        % cost = cost + attractive_gain * 1/exp(x(1,k));
        cost = cost + delta_gain * (du(k)^2);

        % Lateral force constraint
        lateral_force = v^2 * tan(u(:,k)) / L;
        cost = cost + lateral_G_gain * lateral_force^2;

        % Calculate next step wheel positions
        yaw_angle_next = x(3,k+1);
        left_wheel_pos_next = x(1:2,k+1) + [cos(yaw_angle_next), -sin(yaw_angle_next); sin(yaw_angle_next), cos(yaw_angle_next)] * [wb; tw/2];
        right_wheel_pos_next = x(1:2,k+1) + [cos(yaw_angle_next), -sin(yaw_angle_next); sin(yaw_angle_next), cos(yaw_angle_next)] * [wb; -tw/2];
        
        dist_to_obstacle_left_wheel_next = norm((left_wheel_pos_next+left_wheel_pos)./2 - obstacle);
        cost = cost + repulsive_gain * 1/(dist_to_obstacle_left_wheel_next^2 + 1e-3);
    
        % Repulsive potential for right wheel
        dist_to_obstacle_right_wheel_next = norm((right_wheel_pos_next+right_wheel_pos)./2 - obstacle);
        cost = cost + repulsive_gain * 1/(dist_to_obstacle_right_wheel_next^2 + 1e-3);
    end

    % Centering potential
    ref = [x_ref(k);y_ref(k)];
    cost = cost + center_gain * k*sum((ref - x(1:2,k)).^2);
    % cost = cost + 1500*F_pdf(left_wheel_pos(2), ref, 0.2, false);
    % cost = cost + 1500*F_pdf(right_wheel_pos(2), ref, 0.2, false);
    % left_valey = center_gain * ((ref+tw/2 - left_wheel_pos(2))^2 + (ref-tw/2 - left_wheel_pos(2))^2);
    % right_valey = center_gain * ((ref-tw/2 - right_wheel_pos(2))^2 + (ref+tw/2 - right_wheel_pos(2))^2);
    % cost = cost + max(left_valey, right_valey);
    
    % Repulsive potential for the distance between the wheels and the obstacle
    
    % pdf_x = repulsive_gain * F_pdf(left_wheel_pos(1), obstacle(1), obstacle_radius*v, true) * F_pdf(right_wheel_pos(1), obstacle(1), obstacle_radius*v, true);
    % pdf_y = repulsive_gain * F_pdf(left_wheel_pos(2), obstacle(2), obstacle_radius, false) * F_pdf(right_wheel_pos(2), obstacle(2), obstacle_radius, false);
    % cost = cost + pdf_x*pdf_y;

    % left_wheel_pos = x(1:2,k) + [0; tw/2];
    dist_to_obstacle_left_wheel = norm(left_wheel_pos - obstacle);
    cost = cost + repulsive_gain * 1/(dist_to_obstacle_left_wheel^2 + 1e-3);

    % % Repulsive potential for right wheel
    % right_wheel_pos = x(1:2,k) - [0; tw/2];
    dist_to_obstacle_right_wheel = norm(right_wheel_pos - obstacle);
    cost = cost + repulsive_gain * 1/(dist_to_obstacle_right_wheel^2 + 1e-3);


    % dist_to_obstacle = norm(x(1:2,k) - obstacle);
    % cost = cost + repulsive_gain * 1/(dist_to_obstacle^2 + 1e-300);
    
    % x_ave = (x(1:2,k+1) + x(1:2,k))./2;
    % ave_dist_to_obstacle = norm(x_ave - obstacle);
    % cost = cost + repulsive_gain * 1/(ave_dist_to_obstacle^2 + 1e-300);

    % wgheiten for time to collision
    % cost = cost/(x(1,k) + 1e-3);
end
opti.minimize(cost);

% Define the initial and terminal constraints
opti.subject_to(u(1) == u0); % Start at origin
opti.subject_to(du(1) == last_du); % Start at origin
opti.subject_to(x(:,1) == x0); % Start at origin
% opti.subject_to(x(3,end) == 0); % End at goal

% Set initial guess
opti.set_initial(x, [x_ref; y_ref; zeros(1, N+1)]);
du_guess = -pi/5*ones(1, N);
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

% Plot the results
figure("Position",[1000,844,560,157]);
% Plot the lane boundaries
lane_left_boundary = lane_width / 2;
lane_right_boundary = -lane_width / 2;


% Define the grid for the potential field
x_grid = linspace(0, goal(1), 100);
y_grid = linspace(-lane_width, lane_width, 100);
[X, Y] = meshgrid(x_grid, y_grid);

% Calculate the total potential field
total_potential = zeros(size(X));
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        
        left_wheel_pos =  [X(i,j) + wb; Y(i,j) + tw/2];
        right_wheel_pos = [X(i,j) + wb; Y(i,j) - tw/2];

        % Attractive potential
        % attractive_potential = delta_gain * (X(i,j) - goal(1))^2;
        
        % Centering potential
        ref = [X(i,j); interp1(x_ref, y_ref, X(i,j), 'linear', 'extrap')];
        centering_potential = center_gain * (X(i,j)/1.5)*sum(([X(i,j);Y(i,j)] - ref).^2);
        % center_pdf = F_pdf(Y(i,j), ref, 0.2, false);
        % centering_potential_l = center_gain * (left_wheel_pos(2) - (ref+tw/2))^2 + center_gain * (left_wheel_pos(2) - (ref-tw/2))^2;
        % centering_potential_r = center_gain * (right_wheel_pos(2) - (ref-tw/2))^2 + center_gain * (right_wheel_pos(2) - (ref+tw/2))^2;
        % centering_potential = min(centering_potential_l , centering_potential_r);
        
        % Repulsive potential
        dist_to_obstacle = norm([X(i,j); Y(i,j)] - obstacle);
        repulsive_potential = 0.05*repulsive_gain * 1/(dist_to_obstacle^2 + 1e-3);
        % dist_to_obstacle_left_wheel = norm(left_wheel_pos - obstacle);
        % repulsive_potential_x = repulsive_gain * 1/(dist_to_obstacle_left_wheel^2 + 1e-3);
        % dist_to_obstacle_right_wheel = norm(right_wheel_pos - obstacle);
        % repulsive_potential_y = repulsive_gain * 1/(dist_to_obstacle_right_wheel^2 + 1e-3);

        % Total potential
        total_potential(i,j) = centering_potential + repulsive_potential;
    end
end

% Plot the total potential field
% contourf(X, Y, total_potential, 20);
% colorbar;
% xlabel('X');
% ylabel('Y');
% title('Total Potential Field');
% hold on;

% plot the reference trajectory
plot(x_ref, y_ref, 'g--', 'LineWidth', 2); hold on;
% Plot the obstacle
% viscircles(obstacle', obstacle_radius, 'EdgeColor', 'r');

% Plot the lane boundaries
% plot([min(x_grid(:)), max(x_grid(:))], [lane_left_boundary, lane_left_boundary], 'Color', '#FFA500', 'LineWidth', 2);
% plot([min(x_grid(:)), max(x_grid(:))], [lane_right_boundary, lane_right_boundary], 'Color', '#FFA500', 'LineWidth', 2);
% Calculate the lane boundaries considering yaw angle from reference position

% Calculate the lane boundaries considering yaw angle derived from reference position
yaw_ref = atan2(diff(y_ref), diff(x_ref));
yaw_ref = [yaw_ref, yaw_ref(end)]; % Extend the last yaw angle to match the size

boundary_l = zeros(2, N+1);
boundary_r = zeros(2, N+1);
for k = 1:N+1
    yaw_angle = yaw_ref(k);
    boundary_l(:,k) = [x_ref(k);y_ref(k)] + [cos(yaw_ref(k)), -sin(yaw_ref(k)); sin(yaw_ref(k)), cos(yaw_ref(k))] * [0; lane_width/2];
    boundary_r(:,k) = [x_ref(k);y_ref(k)] + [cos(yaw_ref(k)), -sin(yaw_ref(k)); sin(yaw_ref(k)), cos(yaw_ref(k))] * [0; -lane_width/2];
end

% boundary_l = y_ref + [cos(yaw_ref), -sin(yaw_ref); sin(yaw_ref), cos(yaw_ref)] * lane_width/2;
% boundary_r = y_ref - [cos(yaw_ref), -sin(yaw_ref); sin(yaw_ref), cos(yaw_ref)] * lane_width/2;
plot(boundary_l(1,:), boundary_l(2,:), 'Color', '#FFA500', 'LineWidth', 2);
plot(boundary_r(1,:), boundary_r(2,:), 'Color', '#FFA500', 'LineWidth', 2);


plot(x_sol(1,:), x_sol(2,:), 'r-o');
for k = 1:N+1
    yaw_angle = x_sol(3,k);
    rl_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [0; tw/2];
    rr_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [0; -tw/2];
    fl_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [wb; tw/2];
    fr_wheel_pos(:,k) = x_sol(1:2,k) + [cos(yaw_angle), -sin(yaw_angle); sin(yaw_angle), cos(yaw_angle)] * [wb; -tw/2];
end
% rl_wheel_pos = x_sol(1:2,:) + [0; tw/2];
% rr_wheel_pos = x_sol(1:2,:) - [0; tw/2];
plot(rl_wheel_pos(1,:), rl_wheel_pos(2,:), '-b*');
plot(rr_wheel_pos(1,:), rr_wheel_pos(2,:), '-b*');
plot(fl_wheel_pos(1,:), fl_wheel_pos(2,:), '-k*');
plot(fr_wheel_pos(1,:), fr_wheel_pos(2,:), '-k*');
% point of max lateral force
scatter(x_sol(1,idx), x_sol(2,idx), 60, 'magenta', 'filled');
% plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
viscircles(obstacle', obstacle_radius, 'EdgeColor', 'r');
xlabel('\itX \rm[m]');
ylabel('\itY \rm[m]');
% title('2D Path Planning with Potential Field Constraints');
grid on; axis equal;
ylim([min(y_ref)-lane_width, max(y_ref)+lane_width]);
xlim([min(x_ref), max(x_ref)]);
fontname(gcf,"Times New Roman");
fontsize(gca,8,"points");

% figure;
% plot(x_sol(1,1:N), u_sol(1,1:N));
% xlabel('X');
% ylabel('Control Input');
% title('Control Input over Path');

% figure;
% plot(x_sol(1,1:N), x_sol(3,1:N));
% xlabel('X');
% ylabel('Theta');
% title('Theta over Path');