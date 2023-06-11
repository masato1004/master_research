function dx = motion_func(x, u, d, A, B, E)
    % dx = Ax + Bu +Ed
    dx = A*x + B*u + E*d;
end