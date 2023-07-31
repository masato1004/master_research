function dx = runge(x, u, w, A, B, E, dt)
    % x1 = x;      b1 = dt*state_equation(x1, u, d, A, B, E);
    % x2 = x+b1/2; b2 = dt*state_equation(x2, u, d, A, B, E);
    % x3 = x+b2/2; b3 = dt*state_equation(x3, u, d, A, B, E);
    % x4 = x+b3;   b4 = dt*state_equation(x4, u, d, A, B, E);
    x1 = x;      b1 = dt*motion_func(x1, u, w);
    x2 = x+b1/2; b2 = dt*motion_func(x2, u, w);
    x3 = x+b2/2; b3 = dt*motion_func(x3, u, w);
    x4 = x+b3;   b4 = dt*motion_func(x4, u, w);
    dx = x + (b1 + 2*b2 + 2*b3 + b4)/6;
end