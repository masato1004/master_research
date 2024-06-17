function dx = func__rungekutta(x, u, d, A, B, E, dt)
    x1 = x;      b1 = dt*func__motion_eq(x1, u, d, A, B, E);
    x2 = x+b1/2; b2 = dt*func__motion_eq(x2, u, d, A, B, E);
    x3 = x+b2/2; b3 = dt*func__motion_eq(x3, u, d, A, B, E);
    x4 = x+b3;   b4 = dt*func__motion_eq(x4, u, d, A, B, E);
    dx = x + (b1 + 2*b2 + 2*b3 + b4)/6;
end