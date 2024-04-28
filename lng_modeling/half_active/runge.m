function dx = runge(x, u, d, g, A, B, E, G, dt)
    x1 = x;      b1 = dt*motion_func(x1, u, d, g, A, B, E, G);
    x2 = x+b1/2; b2 = dt*motion_func(x2, u, d, g, A, B, E, G);
    x3 = x+b2/2; b3 = dt*motion_func(x3, u, d, g, A, B, E, G);
    x4 = x+b3;   b4 = dt*motion_func(x4, u, d, g, A, B, E, G);
    dx = x + (b1 + 2*b2 + 2*b3 + b4)/6;
end