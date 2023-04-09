% System parameters
M = 0.5;     % mass of the cart
m = 0.2;     % mass of the pendulum
l = 0.3;     % length of the pendulum
g = 9.81;    % acceleration due to gravity

% State space model
A = [0, 1, 0, 0;
     0, 0, -g*m/M, 0;
     0, 0, 0, 1;
     0, 0, (M+m)*g/(l*M), 0];
B = [0; 1/M; 0; -1/(l*M)];
C = eye(4);
D = zeros(4,1);
sys = ss(A,B,C,D);

% Controller gains
Q = diag([10 1 10 1]);
R = 0.1;
K = lqr(A,B,Q,R);
Kff = [0 0 0 0; -m*g/M 0 0 0; 0 0 0 0; (M+m)*g/(l*M) 0 0 0];

% Simulation parameters
t = 0:0.01:5;        % simulation time
x0 = [0; 0; pi+0.1; 0];   % initial condition

% Simulate system with controller
[t,x] = ode45(@(t,x) invpend(x,K,Kff,m,M,l,g), t, x0);

% Plot results
figure;
subplot(2,1,1);
plot(t,x(:,3));
ylabel('\theta (rad)');
title('Inverted Pendulum with Approximation-Free Control');
subplot(2,1,2);
plot(t,x(:,1));
ylabel('x (m)');
xlabel('Time (s)');
