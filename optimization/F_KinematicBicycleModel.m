function dx = F_KinematicBicycleModel(v, x, u, L)
    
    % Kinematic Bicycle Model
    %   x: state vector [x, y, theta]
    %   u: input vector [v, delta]
    %   L: wheelbase
    %   dx: derivative of state vector [dx, dy, dtheta]
    
    % State vector
    x = x(:);
    % v = u(1);
    % delta = u(2);
    % v = 13;
    delta = u;
    beta = atan(0.5*tan(u));
    
    % Derivative of state vector
    % dx = [v*cos(x(3)); v*sin(x(3)); v*tan(delta)/L];
    dx = [v*cos(x(3)+beta); v*sin(x(3)+beta); v*tan(beta)/L];
    
end