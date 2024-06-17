function dx = func__motion_eq(x, u, d, A, B, E)
    % dx = Ax + Bu
    dx = A*x + B*u + E*d;
end