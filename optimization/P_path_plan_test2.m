addpath('./casadi-3.6.7-windows64-matlab2018b')
import casadi.*

% Define the goal position
goal = [10; 10];

% Define the potential field parameters
obstacle = [5; 5];
obstacle_radius = 0.3;
repulsive_gain = 1000;
attractive_gain = 2;

% Define the optimization variables
opti = Opti();
N = 50; % Number of steps
x = opti.variable(2, N+1); % State variables (x, y)
u = opti.variable(2, N); % Control variables (vx, vy)
du = opti.variable(2, N); % Control variables (vx, vy)

% Define the dynamics
dt = 0.1;
for k = 1:N
    opti.subject_to(x(:,k+1) == x(:,k) + dt*u(:,k));
    opti.subject_to(x(1,k+1) >= x(1,k));
    opti.subject_to(x(2,k+1) >= x(2,k));
    if k < N
        opti.subject_to(u(:,k+1) == u(:,k) + du(:,k));
        opti.subject_to((du(1,k))^2 < 2);
        opti.subject_to((du(2,k))^2 < 2);
    end
end

% Define the cost function
cost = 0;
for k = 1:N
    % Attractive potential
    cost = cost + attractive_gain * norm(x(:,k) - goal)^2;
    cost = cost + sum(du(:,k))^2;
    
    % Repulsive potential
    dist_to_obstacle = norm(x(:,k) - obstacle);
    % if dist_to_obstacle < obstacle_radius
    cost = cost + repulsive_gain * (1/dist_to_obstacle - 1/obstacle_radius)^2;
    % end
end
opti.minimize(cost);

% Define the initial and terminal constraints
opti.subject_to(u(:,1) == [0; 0]); % Start at origin
opti.subject_to(x(:,1) == [0; 0]); % Start at origin
opti.subject_to(x(:,end) == goal); % End at goal

% Define the control constraints
% opti.subject_to(-1 <= u <= 1); % Control limits

% Set initial guess
opti.set_initial(x, repmat([0; 0], 1, N+1));
opti.set_initial(u, zeros(2, N));

% Solve the optimization problem
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = true;
opti.solver('ipopt', opts);
% opti.solver('ipopt');
sol = opti.solve();

% Extract the solution
x_sol = sol.value(x);
u_sol = sol.value(u);

% Plot the results
figure;
plot(x_sol(1,:), x_sol(2,:), 'b-o'); hold on;
plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
viscircles(obstacle', obstacle_radius, 'EdgeColor', 'r');
xlabel('X');
ylabel('Y');
title('2D Path Planning with Potential Field Constraints');
grid on;