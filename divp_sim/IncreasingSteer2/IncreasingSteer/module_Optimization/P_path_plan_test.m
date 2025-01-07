addpath('./casadi-3.6.7-windows64-matlab2018b')
import casadi.*

% Define the grid size
grid_size = 45;
[X, Y] = meshgrid(1:0.1:grid_size, 1:0.1:grid_size);

% Define the goal position
goal = [40, 40];

% Define the attractive potential field
attractive_potential = 0.5 * ((X - goal(1)).^2 + (Y - goal(2)).^2);
total_potential = @(x, y) 0.5 * ((x - goal(1)).^2 + (y - goal(2)).^2);
% Define obstacles
obstacles = [20, 20; 30, 30; 25, 40];
obs_pot = zeros(size(X));

% Define the repulsive potential field
obstacle_potential = {};
repulsive_potential = @(X,Y,obstacle) 1./(sqrt((X - obstacle(1)).^2 + (Y - obstacle(2)).^2) + 1e-3);
distance = @(X,Y,obstacle) sqrt((X - obstacle(1)).^2 + (Y - obstacle(2)).^2);
for i = 1:size(obstacles, 1)
    obstacle = obstacles(i, :);
    repulsive = repulsive_potential(X,Y,obstacle); % Add a small value to avoid division by zero
    obstacle_potential{i} = repulsive_potential;
    obs_pot = obs_pot + repulsive;
end

% Combine the attractive and repulsive potentials
test = attractive_potential + obs_pot;

% Plot the potential field
figure;
contourf(X, Y, test);
hold on;
plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot(obstacles(:, 1), obstacles(:, 2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
title('Potential Field for Path Planning');
xlabel('X');
ylabel('Y');
colorbar;
% hold off;

% Define the optimization variables
opti = Opti();
path = opti.variable(2, grid_size*10);

% Define the initial position
start = [10, 10];
plot(start(1), start(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
opti.subject_to(path(:, 1) == start');

% Define the constraints for the path to stay within the grid
opti.subject_to(1 <= path(1, :).*0.1 <= grid_size);
opti.subject_to(1 <= path(2, :).*0.1 <= grid_size);

% Define the objective function based on the potential field
objective = 0;
for t = 1:grid_size*10
    x = path(1, t)*0.1;
    y = path(2, t)*0.1;
    % Ensure x and y are within the bounds of total_potential
    % x = round(x);
    % y = round(y);
    % x = max(1, min(size(total_potential, 1), x));
    % y = max(1, min(size(total_potential, 2), y));

    % Access the value at the corrected indices
    % value = total_potential(x, y);
    objective = objective + total_potential((x), (y));
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        repulsive_potential = obstacle_potential{i};
        % obstacle_potential = obstacle_potential + repulsive_potential;
        objective = objective + repulsive_potential(x, y, obstacles);
        opti.subject_to(distance(x, y, obstacle)>10);
    end
end
opti.minimize(objective);

% Solve the optimization problem
opts = struct;
opts.ipopt.print_level = 0;
opti.solver('ipopt', opts);
tic;
sol = opti.solve();
toc;

% Extract the optimal path
optimal_path = sol.value(path);

% Plot the optimal path
hold on;
plot(optimal_path(1, :).*0.1, optimal_path(2, :).*0.1, 'g-', 'LineWidth', 2);
legend('Goal', 'Obstacles', 'Optimal Path');
hold off;